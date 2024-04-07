`ifndef __MP_AXI_MEMORY_WRITER_INTR_DEFINED__
`define __MP_AXI_MEMORY_WRITER_INTR_DEFINED__


module axi_memory_writer_intr #(
    parameter integer        FREQ_HZ                     = 250000000   ,
    parameter integer        N_BYTES                     = 2           ,
    parameter integer        ADDR_WIDTH                  = 32          ,
    parameter integer        BURST_LIMIT                 = 16          ,
    parameter         [31:0] DEFAULT_MEM_STARTADDR       = 32'h00000000,
    parameter         [31:0] DEFAULT_MEM_HIGHADDR        = 32'h02000000,
    parameter integer        DEFAULT_USER_EVENT_DURATION = 100         ,
    parameter integer        DEFAULT_PORTION_SIZE        = 1048576     ,
    parameter integer        DEFAULT_INTERVAL            = 1000000     ,
    parameter integer        CMD_FIFO_DEPTH              = 64          ,
    parameter string         CMD_FIFO_MEMTYPE            = "block"     ,
    parameter integer        SUSPENDABLE                 = 1           ,
    parameter integer        RETRY_COUNTER_LIMIT         = 250000000    
) (
    input                          aclk             ,
    input                          aresetn          ,
    // CONFIGURATION BUS
    input        [            5:0] S_AXI_AWADDR     ,
    input        [            2:0] S_AXI_AWPROT     ,
    input                          S_AXI_AWVALID    ,
    output logic                   S_AXI_AWREADY    ,
    input        [           31:0] S_AXI_WDATA      ,
    input        [            3:0] S_AXI_WSTRB      ,
    input                          S_AXI_WVALID     ,
    output logic                   S_AXI_WREADY     ,
    output logic [            1:0] S_AXI_BRESP      ,
    output logic                   S_AXI_BVALID     ,
    input                          S_AXI_BREADY     ,
    input        [            5:0] S_AXI_ARADDR     ,
    input        [            2:0] S_AXI_ARPROT     ,
    input                          S_AXI_ARVALID    ,
    output logic                   S_AXI_ARREADY    ,
    output logic [           31:0] S_AXI_RDATA      ,
    output logic [            1:0] S_AXI_RRESP      ,
    output logic                   S_AXI_RVALID     ,
    input                          S_AXI_RREADY     ,
    // USER EVENT INTERRUPTS
    output logic [ ADDR_WIDTH-1:0] CURRENT_ADDR     ,
    output logic [           31:0] TRANSMITTED_BYTES,
    output logic                   USER_EVENT       ,
    input                          USER_EVENT_ACK   ,
    // S_AXIS BUS
    input        [(N_BYTES*8)-1:0] S_AXIS_TDATA     ,
    input                          S_AXIS_TVALID    ,
    input                          S_AXIS_TLAST     ,
    output logic                   S_AXIS_TREADY    ,
    // M_AXI FULL BUS ONLY WRITE MODE
    output logic [ ADDR_WIDTH-1:0] M_AXI_AWADDR     ,
    output logic [            7:0] M_AXI_AWLEN      ,
    output logic [            2:0] M_AXI_AWSIZE     ,
    output logic [            1:0] M_AXI_AWBURST    ,
    output logic                   M_AXI_AWLOCK     ,
    output logic [            3:0] M_AXI_AWCACHE    ,
    output logic [            2:0] M_AXI_AWPROT     ,
    output logic                   M_AXI_AWVALID    ,
    input                          M_AXI_AWREADY    ,
    output logic [(N_BYTES*8)-1:0] M_AXI_WDATA      ,
    output logic [  (N_BYTES)-1:0] M_AXI_WSTRB      ,
    output logic                   M_AXI_WLAST      ,
    output logic                   M_AXI_WVALID     ,
    input                          M_AXI_WREADY     ,
    input        [            1:0] M_AXI_BRESP      ,
    input                          M_AXI_BVALID     ,
    output logic                   M_AXI_BREADY     ,
    input  logic                   UNCORRECT_READ   , 
    output logic ARBITER_RESET 
);

    localparam integer ADDR_LSB            = 2;
    localparam integer ADDR_OPT            = 3;
    localparam integer RESET_COUNTER_LIMIT = 5;
    localparam integer CMD_FIFO_WIDTH = ADDR_WIDTH + 32; // ADDRESS WIDTH OF CURRENT_ADDRESS + TRANSFERRED_SIZE

    logic [11:0][31:0] register;

    logic                  reset_func         ;
    logic [          31:0] portion_size       ;
    logic [ADDR_WIDTH-1:0] mem_startaddr      ;
    logic [ADDR_WIDTH-1:0] mem_highaddr       ;
    logic [          31:0] user_event_duration;

    logic                  queue_overflow     ;
    logic                  queue_overflow_ack ;
    logic                  queue_overflow_flaq ; 

    logic [           1:0] event_dependency       ;
    logic                  run_signal             ;
    logic                  stop_signal            ;
    logic                  irq_ack                ;
    logic                  status                 ;
    logic                  register_read_event    ;

    logic fifo_not_empty   ;
    logic fifo_wren        ;
    logic fifo_wren_impulse;

    logic user_event_ack_internal;

    logic        d_user_event_ack   = 1'b0        ;
    logic        has_user_event_ack = 1'b0        ;
    logic [31:0] valid_count        = '{default:0};

    logic aw_en = 1'b1;

    logic [31:0] reset_counter = '{default:0};

    logic [            31:0] interval              ;
    logic                    interval_using        ;
    logic [            31:0] transmitted_bytes     ;
    logic [(ADDR_WIDTH-1):0] current_address       ;
    logic [            31:0] transmitted_bytes_dout;
    logic [(ADDR_WIDTH-1):0] current_address_dout  ;

    logic suspended;

    logic retry;
    
    
    logic        arb_reset;
    logic [31:0] arb_reset_counter = '{default:0};


    logic [31:0] fifo_current_volume;


    always_comb begin : to_user_logic_assignment_group
        ARBITER_RESET = arb_reset;
        CURRENT_ADDR[ADDR_WIDTH-1:0]  = current_address_dout;
        TRANSMITTED_BYTES             = transmitted_bytes_dout;
        interval_using                = register[1][1];
        event_dependency              = register[1][25:24];
        portion_size                  = register[2];
        mem_highaddr[ADDR_WIDTH-1:0]  = register[3][ADDR_WIDTH-1:0];
        interval                      = register[4]     ;
        mem_startaddr[ADDR_WIDTH-1:0] = register[7][ADDR_WIDTH-1:0];
        user_event_duration           = register[9];
    end 



    always_comb begin : from_usr_logic_assignment_group
        register[1][17]             = suspended;
        register[1][16]             = status;
        register[1][9]              = queue_overflow_flaq;
        register[5]                 = FREQ_HZ;
        register[6]                 = valid_count;
        register[8][ADDR_WIDTH-1:0] = current_address_dout[ADDR_WIDTH-1:0];
        register[10]                = transmitted_bytes_dout;
        register[11]                = fifo_current_volume;

    end 



    always_ff @(posedge aclk) begin : user_event_ack_internal_proc
        case (event_dependency) 
            2'b00   : user_event_ack_internal <= has_user_event_ack;
            2'b01   : user_event_ack_internal <= irq_ack;
            2'b10   : user_event_ack_internal <= register_read_event;
            default : user_event_ack_internal <= has_user_event_ack;
        endcase // event_dependency
    end  



    always_ff @(posedge aclk) begin : user_event_processing 
        if (register[1][26] == 1'b0)
            USER_EVENT <= fifo_not_empty;
        else 
            USER_EVENT <= fifo_wren_impulse;
    end 

    always_ff @(posedge aclk) begin 
        d_user_event_ack <= USER_EVENT_ACK;
    end 

    always_ff @(posedge aclk) begin 
        if (USER_EVENT_ACK & !d_user_event_ack) begin 
            has_user_event_ack <= 1'b1;
        end else begin 
            has_user_event_ack <= 1'b0;
        end 
    end 

    always_ff @(posedge aclk) begin : reset_processing
        if ( !aresetn ) begin 
            reset_func <= 1'b1;
        end else begin 
            if (reset_counter < RESET_COUNTER_LIMIT) begin 
                reset_func <= 1'b1;
            end else begin  
                reset_func <= 1'b0;
            end 
        end 
    end  


    always_ff @(posedge aclk) begin : arb_reset_processing
        if ( !aresetn ) begin 
            arb_reset <= 1'b1;
        end else begin 
            if (arb_reset_counter < RESET_COUNTER_LIMIT) begin 
                arb_reset <= 1'b1;
            end else begin  
                arb_reset <= 1'b0;
            end 
        end 
    end  


    always_ff @(posedge aclk) begin : reset_counter_processing 
        if (!aresetn) begin 
            reset_counter <= '{default:0};
        end else begin 
            if (reset_counter < RESET_COUNTER_LIMIT) begin 
                reset_counter <= reset_counter + 1;
            end else begin 
                if (S_AXI_AWVALID & S_AXI_AWREADY & S_AXI_WVALID & S_AXI_WREADY) begin 
                    if (S_AXI_AWADDR[(ADDR_OPT + ADDR_LSB) : ADDR_LSB] == 'h00) begin 
                        if (S_AXI_WDATA[0]) begin 
                            reset_counter <= {default:0};
                        end 
                    end 
                end 
            end 
        end 
    end 



    always_ff @(posedge aclk) begin : arb_reset_counter_processing 
        if (!aresetn) begin 
            arb_reset_counter <= '{default:0};
        end else begin 
            if (arb_reset_counter < RESET_COUNTER_LIMIT) begin 
                arb_reset_counter <= arb_reset_counter + 1;
            end else begin 
                if (S_AXI_AWVALID & S_AXI_AWREADY & S_AXI_WVALID & S_AXI_WREADY) begin 
                    if (S_AXI_AWADDR[(ADDR_OPT + ADDR_LSB) : ADDR_LSB] == 'h00) begin 
                        if (S_AXI_WDATA[1]) begin 
                            arb_reset_counter <= {default:0};
                        end 
                    end 
                end 
            end 
        end 
    end 




    always_ff @(posedge aclk) begin : run_signal_proc 
        if (!aresetn | reset_func ) begin 
            run_signal <= 1'b0;
        end else begin 
            if (S_AXI_AWVALID & S_AXI_AWREADY & S_AXI_WVALID & S_AXI_WREADY) begin
                if (S_AXI_AWADDR[(ADDR_OPT + ADDR_LSB) : ADDR_LSB] == 'h01) begin
                    run_signal <= S_AXI_WDATA[0];
                end else begin 
                    run_signal <= 1'b0;
                end 
            end else begin 
                run_signal <= 1'b0;
            end 
        end  
    end 



    always_ff @(posedge aclk) begin : stop_signal_proc 
        if (!aresetn | reset_func ) begin 
            stop_signal <= 1'b0;
        end else begin 
            if (S_AXI_AWVALID & S_AXI_AWREADY & S_AXI_WVALID & S_AXI_WREADY) begin
                if (S_AXI_AWADDR[(ADDR_OPT + ADDR_LSB) : ADDR_LSB] == 'h01) begin
                    stop_signal <= ~S_AXI_WDATA[0];
                end else begin 
                    stop_signal <= 1'b0;
                end 
            end else begin 
                stop_signal <= 1'b0;
            end 
        end  
    end 

    

    always_ff @(posedge aclk) begin : irq_ack_proc 
        if (!aresetn) begin  
            irq_ack <= 1'b0;
        end else begin 
            if (S_AXI_AWVALID & S_AXI_AWREADY & S_AXI_WVALID & S_AXI_WREADY) begin
                if (S_AXI_AWADDR[(ADDR_OPT + ADDR_LSB) : ADDR_LSB] == 'h01) begin
                    irq_ack <= S_AXI_WDATA[8];
                end else begin  
                    irq_ack <= 1'b0;
                end 
            end else begin 
                irq_ack <= 1'b0;
            end 
        end 
    end 



    always_ff @(posedge aclk) begin : queue_overflow_ack_proc
        if (!aresetn | reset_func ) begin
            queue_overflow_ack <= 1'b0;
        end else begin
            if (S_AXI_AWVALID & S_AXI_AWREADY & S_AXI_WVALID & S_AXI_WREADY) begin
                if (S_AXI_AWADDR[(ADDR_OPT + ADDR_LSB) : ADDR_LSB] == 'h01) begin
                    queue_overflow_ack <= S_AXI_WDATA[9];
                end else begin
                    queue_overflow_ack <= 1'b0;
                end
            end else begin
                queue_overflow_ack <= 1'b0;
            end
        end
    end 



    always_ff @(posedge aclk) begin : queue_overflow_flaq_proc
        if (!aresetn | reset_func ) begin
            queue_overflow_flaq <= 1'b0;
        end else begin
            if (fifo_wren) begin 
                if (queue_overflow) begin 
                    queue_overflow_flaq <= 1'b1;
                end else begin 
                    queue_overflow_flaq <= queue_overflow_flaq;
                end
            end else begin 
                if (queue_overflow_ack) begin 
                    queue_overflow_flaq <= 1'b0;
                end else begin 
                    queue_overflow_flaq <= queue_overflow_flaq;
                end 
            end 

        end
    end 



    always_ff @(posedge aclk) begin : register_read_event_proc
        if (!aresetn) begin
            register_read_event <= 1'b0;
        end else begin
            if (S_AXI_ARVALID & S_AXI_ARREADY & ~S_AXI_RVALID) begin
                case (S_AXI_ARADDR[(ADDR_OPT + ADDR_LSB) : ADDR_LSB])
                    'h8     : register_read_event <= 1'b1;
                    default : register_read_event <= 1'b0;
                endcase // S_AXI_ARADDR
            end else begin 
                register_read_event <= 1'b0;
            end 
        end
    end 


    /**/
    always_ff @(posedge aclk) begin : aw_en_processing 
        if (!aresetn) 
            aw_en <= 1'b1;
        else
            if (!S_AXI_AWREADY & S_AXI_AWVALID & S_AXI_WVALID & aw_en)
                aw_en <= 1'b0;
            else
                if (S_AXI_BREADY & S_AXI_BVALID)
                    aw_en <= 1'b1;
    end 


    /**/
    always_ff @(posedge aclk) begin : S_AXI_AWREADY_processing 
        if (!aresetn)
            S_AXI_AWREADY <= 1'b0;
        else
            if (!S_AXI_AWREADY & S_AXI_AWVALID & S_AXI_WVALID & aw_en)
                S_AXI_AWREADY <= 1'b1;
            else 
                S_AXI_AWREADY <= 1'b0;
    end 



    always_ff @(posedge aclk) begin : S_AXI_WREADY_processing 
        if (!aresetn)
            S_AXI_WREADY <= 1'b0;
        else
            if (!S_AXI_WREADY & S_AXI_WVALID & S_AXI_AWVALID & aw_en)
                S_AXI_WREADY <= 1'b1;
            else
                S_AXI_WREADY <= 1'b0;

    end 



    always_ff @(posedge aclk) begin : S_AXI_BVALID_processing
        if (!aresetn)
            S_AXI_BVALID <= 1'b0;
        else
            // if (S_AXI_AWVALID & S_AXI_AWREADY & S_AXI_WVALID & S_AXI_WREADY & ~S_AXI_BVALID)
            if (S_AXI_WVALID & S_AXI_WREADY & S_AXI_AWVALID & S_AXI_AWREADY & ~S_AXI_BVALID)
                S_AXI_BVALID <= 1'b1;
            else
                if (S_AXI_BVALID & S_AXI_BREADY)
                    S_AXI_BVALID <= 1'b0;

    end 



    always_ff @(posedge aclk) begin : S_AXI_ARREADY_processing 
        if (!aresetn)
            S_AXI_ARREADY <= 1'b0;
        else
            if (!S_AXI_ARREADY & S_AXI_ARVALID)
                S_AXI_ARREADY <= 1'b1;
            else
                S_AXI_ARREADY <= 1'b0;
            
    end



    always_ff @(posedge aclk) begin : S_AXI_RVALID_processing
        if (!aresetn)
            S_AXI_RVALID <= 1'b0;
        else
            if (S_AXI_ARVALID & S_AXI_ARREADY & ~S_AXI_RVALID)
                S_AXI_RVALID <= 1'b1;
            else 
                if (S_AXI_RVALID & S_AXI_RREADY)
                    S_AXI_RVALID <= 1'b0;

    end 


    always_ff @(posedge aclk) begin : S_AXI_RDATA_processing
        if (!aresetn)
            S_AXI_RDATA <= '{default:0};
        else
            if (S_AXI_ARVALID & S_AXI_ARREADY & ~S_AXI_RVALID)
                case (S_AXI_ARADDR[(ADDR_OPT + ADDR_LSB) : ADDR_LSB])
                    'h0     : S_AXI_RDATA <= register[0];
                    'h1     : S_AXI_RDATA <= register[1];
                    'h2     : S_AXI_RDATA <= register[2];
                    'h3     : S_AXI_RDATA <= register[3];
                    'h4     : S_AXI_RDATA <= register[4];
                    'h5     : S_AXI_RDATA <= register[5];
                    'h6     : S_AXI_RDATA <= register[6];
                    'h7     : S_AXI_RDATA <= register[7];
                    'h8     : S_AXI_RDATA <= register[8];
                    'h9     : S_AXI_RDATA <= register[9];
                    'hA     : S_AXI_RDATA <= register[10];
                    'hB     : S_AXI_RDATA <= register[11];
                    default : S_AXI_RDATA <= S_AXI_RDATA;
                endcase // S_AXI_ARADDR
    end 



    always_ff @(posedge aclk) begin : S_AXI_RRESP_processing 
        if (!aresetn) 
            S_AXI_RRESP <= '{default:0};
        else
            if (S_AXI_ARVALID & S_AXI_ARREADY & ~S_AXI_RVALID)
                case (S_AXI_ARADDR[(ADDR_OPT + ADDR_LSB) : ADDR_LSB])
                    'h0 : S_AXI_RRESP <= '{default:0};
                    'h1 : S_AXI_RRESP <= '{default:0};
                    'h2 : S_AXI_RRESP <= '{default:0};
                    'h3 : S_AXI_RRESP <= '{default:0};
                    'h4 : S_AXI_RRESP <= '{default:0};
                    'h5 : S_AXI_RRESP <= '{default:0};
                    'h6 : S_AXI_RRESP <= '{default:0};
                    'h7 : S_AXI_RRESP <= '{default:0};
                    'h8 : S_AXI_RRESP <= '{default:0};
                    'h9 : S_AXI_RRESP <= '{default:0};
                    'hA : S_AXI_RRESP <= '{default:0};
                    'hB : S_AXI_RRESP <= '{default:0};
                    default : S_AXI_RRESP <= 'b10;
                endcase; // S_AXI_ARADDR
    end                     



    always_ff @(posedge aclk) begin : S_AXI_BRESP_processing
        if (!aresetn)
            S_AXI_BRESP <= '{default:0};
        else
            if (S_AXI_AWVALID & S_AXI_AWREADY & S_AXI_WVALID & S_AXI_WREADY & ~S_AXI_BVALID)
                if (S_AXI_AWADDR >= 0 | S_AXI_AWADDR <= 10 )
                    S_AXI_BRESP <= '{default:0};
                else
                    S_AXI_BRESP <= 'b10;
    end



    always_ff @(posedge aclk) begin : reg_0_processing
        if (!aresetn) 
            register[0] <= 'b0;
        else
            if (S_AXI_AWVALID & S_AXI_AWREADY & S_AXI_WVALID & S_AXI_WREADY)
                if (S_AXI_AWADDR[(ADDR_OPT + ADDR_LSB) : ADDR_LSB] == 'h00)
                    register[0] <= S_AXI_WDATA;
    end 


    /*done*/
    always_ff @(posedge aclk) begin : reg_1_processing
        if (!aresetn) begin
            register[1][8:0]   <= 'b0;
            register[1][15:10] <= 'b0;
            register[1][31:18] <= 'b0;
        end else
        if (S_AXI_AWVALID & S_AXI_AWREADY & S_AXI_WVALID & S_AXI_WREADY)
            if (S_AXI_AWADDR[(ADDR_OPT + ADDR_LSB) : ADDR_LSB] == 'h01) begin
                register[1][7:1]   <= S_AXI_WDATA[7:1];
                register[1][15:10] <= S_AXI_WDATA[15:10];
                register[1][31:18] <= S_AXI_WDATA[31:18];
            end
    end 


    /*done*/
    always_ff @(posedge aclk) begin : reg_2_processing 
        if (!aresetn)
            register[2] <= DEFAULT_PORTION_SIZE;
        else
            if (S_AXI_AWVALID & S_AXI_AWREADY & S_AXI_WVALID & S_AXI_WREADY)
                if (S_AXI_AWADDR[(ADDR_OPT + ADDR_LSB) : ADDR_LSB] == 'h02)
                    register[2] <= S_AXI_WDATA;
    end 



    always_ff @(posedge aclk) begin : reg_3_processing 
        if (!aresetn)
            register[3] <= DEFAULT_MEM_HIGHADDR;
        else
            if (S_AXI_AWVALID & S_AXI_AWREADY & S_AXI_WVALID & S_AXI_WREADY)
                if (S_AXI_AWADDR[(ADDR_OPT + ADDR_LSB) : ADDR_LSB] == 'h03)
                    register[3] <= S_AXI_WDATA;
    end 



    always_ff @(posedge aclk) begin : reg_4_processing 
        if (!aresetn)
            register[4] <= DEFAULT_INTERVAL;
        else
            if (S_AXI_AWVALID & S_AXI_AWREADY & S_AXI_WVALID & S_AXI_WREADY)
                if (S_AXI_AWADDR[(ADDR_OPT + ADDR_LSB) : ADDR_LSB] == 'h04)
                    register[4] <= S_AXI_WDATA;
    end 



    always_ff @(posedge aclk) begin : reg_7_processing 
        if (!aresetn)
            register[7] <= DEFAULT_MEM_STARTADDR;
        else
            if (S_AXI_AWVALID & S_AXI_AWREADY & S_AXI_WVALID & S_AXI_WREADY)
                if (S_AXI_AWADDR[(ADDR_OPT + ADDR_LSB) : ADDR_LSB] == 'h07)
                    register[7] <= S_AXI_WDATA;
    end 



    always_ff @(posedge aclk) begin : reg_9_processing 
        if (!aresetn)
            register[9] <= DEFAULT_USER_EVENT_DURATION;
        else
            if (S_AXI_AWVALID & S_AXI_AWREADY & S_AXI_WVALID & S_AXI_WREADY)
                if (S_AXI_AWADDR[(ADDR_OPT + ADDR_LSB) : ADDR_LSB] == 'h09)
                    register[9] <= S_AXI_WDATA;
    end 


    axi_memory_writer_intr_functional #(
        .DATA_WIDTH (N_BYTES*8  ),
        .ADDR_WIDTH (ADDR_WIDTH ),
        .BURST_LIMIT(BURST_LIMIT),
        .SUSPENDABLE(SUSPENDABLE)
    ) axi_memory_writer_intr_functional_inst (
        .CLK              (aclk             ),
        .RESET            (reset_func       ),
        .MEM_BASEADDR     (mem_startaddr    ),
        .MEM_HIGHADDR     (mem_highaddr     ),
        .MEM_PORTION_SIZE (portion_size     ),
        .INTERVAL         (interval         ),
        .SUSPENDED        (suspended),
        .OVERFLOW         (queue_overflow   ), // used when SUSPENDABLE = 1
        
        .TRANSMITTED_BYTES(transmitted_bytes),
        .CURRENT_ADDRESS  (current_address  ),
        .TRANSMITTED_FLAQ (fifo_wren        ),
        
        .RUN              (run_signal       ),
        .STOP             (stop_signal      ),
        .INTERVAL_USING   (interval_using   ),
        .STATUS           (status           ),
        .S_AXIS_TDATA     (S_AXIS_TDATA     ),
        .S_AXIS_TVALID    (S_AXIS_TVALID    ),
        .S_AXIS_TLAST     (S_AXIS_TLAST     ),
        .S_AXIS_TREADY    (S_AXIS_TREADY    ),
        .M_AXI_AWADDR     (M_AXI_AWADDR     ),
        .M_AXI_AWLEN      (M_AXI_AWLEN      ),
        .M_AXI_AWSIZE     (M_AXI_AWSIZE     ),
        .M_AXI_AWBURST    (M_AXI_AWBURST    ),
        .M_AXI_AWLOCK     (M_AXI_AWLOCK     ),
        .M_AXI_AWCACHE    (M_AXI_AWCACHE    ),
        .M_AXI_AWPROT     (M_AXI_AWPROT     ),
        .M_AXI_AWVALID    (M_AXI_AWVALID    ),
        .M_AXI_AWREADY    (M_AXI_AWREADY    ),
        .M_AXI_WDATA      (M_AXI_WDATA      ),
        .M_AXI_WSTRB      (M_AXI_WSTRB      ),
        .M_AXI_WLAST      (M_AXI_WLAST      ),
        .M_AXI_WVALID     (M_AXI_WVALID     ),
        .M_AXI_WREADY     (M_AXI_WREADY     ),
        .M_AXI_BRESP      (M_AXI_BRESP      ),
        .M_AXI_BVALID     (M_AXI_BVALID     ),
        .M_AXI_BREADY     (M_AXI_BREADY     )
    );


    fifo_cmd_sync_xpm #(
        .DATA_WIDTH(CMD_FIFO_WIDTH  ),
        .MEMTYPE   (CMD_FIFO_MEMTYPE),
        .DEPTH     (CMD_FIFO_DEPTH  )
    ) fifo_cmd_sync_xpm_inst (
        .CLK  (aclk                                                                ),
        .RESET(reset_func                                                          ),
        .DIN  ({current_address[ADDR_WIDTH-1:0], transmitted_bytes[31:0]}          ),
        .WREN (fifo_wren                                                           ),
        .FULL (queue_overflow                                                      ),
        .DOUT ({current_address_dout[ADDR_WIDTH-1:0], transmitted_bytes_dout[31:0]}),
        .RDEN (user_event_ack_internal                                             ),
        .EMPTY(fifo_empty                                                          )
    );


    always_comb begin 
        fifo_not_empty = ~fifo_empty;
    end 

    logic [63:0] time_marker = '{default:0};

    always_ff @(posedge aclk) begin 
        if (~aresetn) begin 
            time_marker <= '{default:0};
        end else begin 
            time_marker <= time_marker + 1;
        end  
    end 

    logic uncorrect_read_latch;

    always_ff @(posedge aclk) begin 
        if (~aresetn | reset_func) begin 
            uncorrect_read_latch <= 1'b0;
        end else begin 
            if (UNCORRECT_READ) begin 
                uncorrect_read_latch <= 1'b1;
            end else begin 
                uncorrect_read_latch <= uncorrect_read_latch;
            end 
        end 
    end 

    irq_generator irq_generator_inst (
        .CLK           (aclk               ),
        .RESET         (reset_func         ),
        .RETRY         (retry              ),
        .USER_EVENT_IN (fifo_wren          ),
        .USER_EVENT_OUT(fifo_wren_impulse  ),
        .DURATION      (user_event_duration)
    );

    irq_retryer #(
        .RETRY_COUNTER_LIMIT(RETRY_COUNTER_LIMIT),
        .CMD_FIFO_DEPTH     (CMD_FIFO_DEPTH     )
    ) irq_retryer_inst (
        .CLK           (aclk                   ),
        .RESET         (reset_func             ),
        .USER_EVENT_IN (fifo_wren              ),
        .USER_EVENT_ACK(user_event_ack_internal),
        .RETRY         (retry                  )
    );


    always_ff @(posedge aclk) begin : fifo_current_volume_processing
        if (reset_func) begin
            fifo_current_volume <= '{default:0};
        end else begin
            if (fifo_wren) begin
                if (user_event_ack_internal) begin
                    fifo_current_volume <= fifo_current_volume;
                end else begin
                    if (queue_overflow) begin 
                        fifo_current_volume <= fifo_current_volume;
                    end else begin 
                        fifo_current_volume <= fifo_current_volume + 1;
                    end 
                end
            end else begin
                if (user_event_ack_internal & !fifo_empty) begin
                    fifo_current_volume <= fifo_current_volume - 1;
                end else begin
                    fifo_current_volume <= fifo_current_volume;
                end
            end
        end
    end 


endmodule
`endif //__MP_AXI_MEMORY_WRITER_INTR_DEFINED__