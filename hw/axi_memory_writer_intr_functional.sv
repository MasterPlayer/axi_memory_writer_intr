`timescale 1ns / 1ps



module axi_memory_writer_intr_functional #(
    parameter integer DATA_WIDTH  = 32,
    parameter integer ADDR_WIDTH  = 32,
    parameter integer BURST_LIMIT = 16,
    parameter integer SUSPENDABLE = 1
) (
    input  logic                      CLK              ,
    input  logic                      RESET            ,
    input  logic [    ADDR_WIDTH-1:0] MEM_BASEADDR     ,
    input  logic [    ADDR_WIDTH-1:0] MEM_HIGHADDR     ,
    input  logic [              31:0] MEM_PORTION_SIZE ,
    input  logic [              31:0] INTERVAL         ,
    input  logic                      INTERVAL_USING   ,
    input  logic                      OVERFLOW         , // used when SUSPENDABLE = 1
    output logic                      SUSPENDED        ,
    output logic [              31:0] TRANSMITTED_BYTES,
    output logic [    ADDR_WIDTH-1:0] CURRENT_ADDRESS  ,
    output logic                      TRANSMITTED_FLAQ ,
    input  logic                      RUN              ,
    input  logic                      STOP             ,
    output logic                      STATUS           ,
    input  logic [    DATA_WIDTH-1:0] S_AXIS_TDATA     ,
    input  logic                      S_AXIS_TVALID    ,
    input  logic                      S_AXIS_TLAST     ,
    output logic                      S_AXIS_TREADY    ,
    output logic [    ADDR_WIDTH-1:0] M_AXI_AWADDR     ,
    output logic [               7:0] M_AXI_AWLEN      ,
    output logic [               2:0] M_AXI_AWSIZE     ,
    output logic [               1:0] M_AXI_AWBURST    ,
    output logic                      M_AXI_AWLOCK     ,
    output logic [               3:0] M_AXI_AWCACHE    ,
    output logic [               2:0] M_AXI_AWPROT     ,
    output logic                      M_AXI_AWVALID    ,
    input  logic                      M_AXI_AWREADY    ,
    output logic [    DATA_WIDTH-1:0] M_AXI_WDATA      ,
    output logic [(DATA_WIDTH/8)-1:0] M_AXI_WSTRB      ,
    output logic                      M_AXI_WLAST      ,
    output logic                      M_AXI_WVALID     ,
    input  logic                      M_AXI_WREADY     ,
    input  logic [               1:0] M_AXI_BRESP      ,
    input  logic                      M_AXI_BVALID     ,
    output logic                      M_AXI_BREADY
);

    localparam integer FIFO_DEPTH             = (BURST_LIMIT*2)       ;
    localparam integer C_AXSIZE_INT           = $clog2((DATA_WIDTH/8));
    localparam integer INCREMENT_VALUE        = (DATA_WIDTH/8)        ;
    localparam integer ACCUMULATE_BURST_LIMIT = (BURST_LIMIT*4)       ;

    logic has_run_initiated  = 'b0;
    logic has_runned         = 'b0;
    logic has_stop_initiated = 'b0;
    logic has_bresp_flaq     = 'b0;
    logic has_active_packet  = 'b0;

    logic has_fifo_word_count_reached = 'b0;

    typedef enum {
        IDLE_ST             ,
        WAIT_FOR_DATA_ST    ,
        ESTABLISH_ADDR_ST   , 
        WRITE_ST            ,
        WAIT_BRESP_ST       ,
        SUSPENDED_ST        
    } fsm;

    fsm          current_state      = IDLE_ST     ;
    logic        fifo_reset         = 'b0         ;
    logic        write_ability      = 'b0         ;
    logic        in_dout_last                     ;
    logic        in_rden            = 'b0         ;
    logic        in_empty                         ;
    logic [31:0] fifo_data_count                  ;
    logic [31:0] fifo_word_count                  ;
    logic [ 7:0] awlen_counter    = '{default:0};
    logic [31:0] word_counter       = '{default:0};
    logic [ 8:0] awlen_reg          = '{default:0};

    // logic [$clog2(BURST_LIMIT)-1:0] burst_accumulate_timer             = '{default:0};
    logic [$clog2(ACCUMULATE_BURST_LIMIT)-1:0] burst_accumulate_timer             = '{default:0};
    logic                                      has_burst_accumulate_timer_reached = 1'b0        ;

    logic [12:0] bytes_to_bound = '{default:0};
    logic [12:0] words_to_bound = '{default:0};

    logic addr_limit_reached         = 'b0; // current_address pointer setupped greater than HIGHADDR value
    logic allow_address_crossing_reg = 'b0;

    logic unfound_last_flaq = 'b0;

    logic found_last_flaq = 'b0;

    logic [31:0] address_increment = '{default:0};

    logic fifo_gt_burst  = 1'b0;
    logic word_gt_burst  = 1'b0;
    logic bound_gt_burst = 1'b0;
    logic bound_gt_word  = 1'b0;
    logic word_gt_fifo   = 1'b0;
    logic bound_gt_fifo  = 1'b0;

    logic word_counter_zero_flaq = 1'b0;

    logic [31:0] interval_timer   = '{default:0};
    logic        interval_work    = 1'b0        ;
    logic        interval_reached = 1'b0        ;

    logic uncompleted_packet = 1'b0;

    always_comb begin
        M_AXI_AWSIZE  = C_AXSIZE_INT;
        M_AXI_AWCACHE = '{default:0};
        M_AXI_AWPROT  = '{default:0};
        M_AXI_AWLOCK  = 1'b0;
        M_AXI_AWBURST = 2'b01;
        // M_AXI_BREADY  = 1'b1;
    end 

    always_ff @(posedge CLK) begin : SUSPENDED_processing 
        if (current_state == SUSPENDED_ST) begin
            SUSPENDED <= 1'b1;
        end else begin
            SUSPENDED <= 1'b0;
        end
    end 

    always_comb begin 
        if (portion_end_flaq) begin 
            M_AXI_WSTRB   = '{default:0};
        end else begin 
            M_AXI_WSTRB   = '{default:1};
        end 
    end 


    always_comb begin : M_AXI_WLAST_processing 

        if ( (current_state == WRITE_ST)  & (awlen_counter == M_AXI_AWLEN))
            M_AXI_WLAST = 'b1;
        else 
            M_AXI_WLAST = 'b0;
    end 



    always_ff @(posedge CLK) begin : status_processing
        case (current_state)
        
            IDLE_ST :
                STATUS <= 1'b0;

            default:
                STATUS <= 1'b1;

        endcase
    end



    always_ff @(posedge CLK) begin : has_stop_initiated_processing
        if (RESET) begin 
            has_stop_initiated <= 1'b0;
        end else begin 
            case (current_state)

                IDLE_ST :
                    has_stop_initiated <= 1'b0;

                default:
                    if (STOP) begin 
                        has_stop_initiated <= 1'b1;
                    end else begin 
                        has_stop_initiated <= has_stop_initiated;
                    end

            endcase
        end
    end



    always_ff @(posedge CLK) begin : has_run_initiated_processing 
        if (RESET) begin 
            has_run_initiated <= 1'b0;
        end else begin 
            case (current_state) 
                IDLE_ST :
                    if (RUN) begin 
                        has_run_initiated <= 1'b1;
                    end else begin  
                        has_run_initiated <= has_run_initiated;
                    end 

                default : 
                    has_run_initiated <= 1'b0;

            endcase
        end 
    end 


    always_ff @(posedge CLK) begin : has_runned_processing 
        if (RESET | has_stop_initiated) begin 
            has_runned <= 1'b0;
        end else begin 
            if (has_run_initiated & !has_active_packet) begin 
                has_runned <= 1'b1;
            end else begin 
                has_runned <= has_runned;
            end 
        end 
    end 



    always_ff @(posedge CLK) begin : word_counter_zero_flaq_processing 
        if (word_counter == 0) begin 
            word_counter_zero_flaq <= 1'b1;
        end else begin 
            word_counter_zero_flaq <= 1'b0;
        end 
    end 



    generate
        
        if (C_AXSIZE_INT > 0) begin : word_counter_processing_N_BYTES 

            always_ff @(posedge CLK) begin : word_counter_processing
                case (current_state)

                    IDLE_ST :
                        if (MEM_PORTION_SIZE[(C_AXSIZE_INT-1):0] == 0) begin
                            word_counter <= MEM_PORTION_SIZE[31:C_AXSIZE_INT];
                        end else begin
                            word_counter <= MEM_PORTION_SIZE[31:C_AXSIZE_INT] + 1;
                        end

                    WRITE_ST :
                        if (M_AXI_WVALID & M_AXI_WREADY) begin 
                            if (awlen_counter == M_AXI_AWLEN & !word_counter_zero_flaq) begin 
                        //     // (awlen_counter == M_AXI_AWLEN)
                        //     // if (M_AXI_WVALID & M_AXI_WREADY & M_AXI_WLAST & (|word_counter)) begin 
                                word_counter <= word_counter - awlen_reg;
                            end else begin 
                                word_counter <= word_counter;
                            end
                        end else begin 
                            word_counter <= word_counter;
                        end 

                    default:
                        word_counter <= word_counter;

                endcase
            end
        end else begin 

            always_ff @(posedge CLK) begin : word_counter_processing
                case (current_state)

                    IDLE_ST :
                        word_counter <= MEM_PORTION_SIZE[31:C_AXSIZE_INT];

                    WRITE_ST :
                        if (M_AXI_WVALID & M_AXI_WREADY) begin 
                            if (awlen_counter == M_AXI_AWLEN & !word_counter_zero_flaq) begin 
                                word_counter <= word_counter - awlen_reg;
                            end else begin 
                                word_counter <= word_counter;
                            end
                        end else begin 
                            word_counter <= word_counter;
                        end 


                    default:
                        word_counter <= word_counter;

                endcase
            end

        end 

    endgenerate


    
    always_ff @(posedge CLK) begin : address_increment_processing 
        case (current_state) 
            WRITE_ST : 
                if (M_AXI_WVALID & M_AXI_WREADY) begin 
                    address_increment <= address_increment + 1;
                end else begin 
                    address_increment <= address_increment;
                end 

            WAIT_BRESP_ST : 
                if (has_bresp_flaq) begin 
                    address_increment <= '{default:0};
                end else begin 
                    address_increment <= address_increment;
                end 

            default : 
                address_increment <= address_increment;


        endcase // current_state
    end 



    // byte addressation
    always_ff @(posedge CLK) begin : m_axi_awaddr_processing
        if (RESET | allow_address_crossing_reg | (interval_reached & addr_limit_reached)) begin 
            M_AXI_AWADDR <= MEM_BASEADDR;
        end else begin 
            case (current_state)

                WAIT_BRESP_ST : 
                    if (has_bresp_flaq) begin 
                        M_AXI_AWADDR <= M_AXI_AWADDR + (address_increment << C_AXSIZE_INT);
                    end else begin 
                        M_AXI_AWADDR <= M_AXI_AWADDR;
                    end 

                default:
                    M_AXI_AWADDR <= M_AXI_AWADDR;

            endcase
        end 
    end



    always_ff @(posedge CLK) begin : addr_limit_reached_processing 
        if (MEM_HIGHADDR <= M_AXI_AWADDR) begin 
            addr_limit_reached <= 1'b1;
        end else begin 
            addr_limit_reached <= 1'b0;
        end 
    end 



    always_comb begin 
        allow_address_crossing_reg = has_bresp_flaq & addr_limit_reached & ((found_last_flaq | portion_end_flaq) & word_counter_zero_flaq);
    end 



    always_ff @(posedge CLK) begin : has_fifo_word_count_reached_processing 
        if (fifo_word_count > BURST_LIMIT) begin 
            has_fifo_word_count_reached <= 1'b1;
        end else begin 
            has_fifo_word_count_reached <= 1'b0;
        end 
    end 



    always_ff @(posedge CLK) begin : fifo_gt_burst_processing 
        if (fifo_word_count > BURST_LIMIT) begin 
            fifo_gt_burst <= 1'b1;
        end else begin 
            fifo_gt_burst <= 1'b0;
        end 
    end 

    always_ff @(posedge CLK) begin : word_gt_burst_processing 
        if (word_counter > BURST_LIMIT) begin 
            word_gt_burst <= 1'b1;
        end else begin 
            word_gt_burst <= 1'b0;
        end
    end 

    always_ff @(posedge CLK) begin : bound_gt_burst_processing 
        if (words_to_bound < BURST_LIMIT) begin 
            bound_gt_burst <= 1'b0;
        end else begin 
            bound_gt_burst <= 1'b1;
        end 
    end 

    always_ff @(posedge CLK) begin : bound_gt_word_processing 
        if (words_to_bound < word_counter) begin 
            bound_gt_word <= 1'b0;
        end else begin 
            bound_gt_word <= 1'b1;
        end
    end 


    always_ff @(posedge CLK) begin : word_gt_fifo_processing 
        if (word_counter > fifo_word_count) begin 
            word_gt_fifo <= 1'b1;
        end else begin 
            word_gt_fifo <= 1'b0;
        end 
    end 


    always_ff @(posedge CLK) begin : bound_gt_fifo_processing 
        if (words_to_bound > fifo_word_count) begin 
            bound_gt_fifo <= 1'b1;
        end else begin 
            bound_gt_fifo <= 1'b0;
        end 
    end 



    always_ff @(posedge CLK) begin : awlen_reg_processing
        case (current_state)
            ESTABLISH_ADDR_ST:
                if (fifo_gt_burst) begin 
                    if (word_gt_burst | word_counter_zero_flaq) begin 
                        if (bound_gt_burst) begin 
                            awlen_reg <= (BURST_LIMIT);
                        end else begin 
                            awlen_reg <= words_to_bound;
                        end 
                    end else begin 
                        if (bound_gt_word) begin 
                            awlen_reg <= word_counter[8:0];
                        end else begin 
                            awlen_reg <= words_to_bound;
                        end 
                    end 
                end else begin 
                    if (word_gt_fifo | word_counter_zero_flaq) begin
                        if (bound_gt_fifo) begin 
                            awlen_reg <= fifo_word_count;
                        end else begin 
                            awlen_reg <= words_to_bound;
                        end 
                    end else begin 
                        if (bound_gt_word) begin 
                            awlen_reg <= word_counter[8:0];
                        end else begin 
                            awlen_reg <= words_to_bound;
                        end 
                    end 
                end 
      
            default : 
                awlen_reg <= awlen_reg;

        endcase
    end



    // Придумать как схема будет себя вести, если word_counter = 0;
    always_ff @(posedge CLK) begin : M_AXI_AWLEN_processing 
        if (RESET) begin 
            M_AXI_AWLEN <= '{default:0};
        end else begin 

            case (current_state)
                ESTABLISH_ADDR_ST : 
                    if (fifo_gt_burst) begin 
                        if (word_gt_burst | word_counter_zero_flaq) begin 
                            if (bound_gt_burst) begin 
                                M_AXI_AWLEN <= (BURST_LIMIT-1);
                            end else begin 
                                M_AXI_AWLEN <= (words_to_bound-1);
                            end 
                        end else begin 
                            if (bound_gt_word) begin 
                                M_AXI_AWLEN <= word_counter[7:0] - 1;
                            end else begin 
                                M_AXI_AWLEN <= (words_to_bound-1);
                            end 
                        end 
                    end else begin 
                        if (word_gt_fifo | word_counter_zero_flaq) begin
                            if (bound_gt_fifo) begin 
                                M_AXI_AWLEN <= fifo_word_count-1;
                            end else begin 
                                M_AXI_AWLEN <= words_to_bound-1;
                            end 
                        end else begin 
                            if (bound_gt_word) begin 
                                M_AXI_AWLEN <= word_counter[7:0] - 1;
                            end else begin 
                                M_AXI_AWLEN <= words_to_bound-1;
                            end 
                        end 
                    end 

                default : 
                    M_AXI_AWLEN <= M_AXI_AWLEN;

            endcase // current_state
        end 
    end 



    always_ff @(posedge CLK) begin : m_axi_awvalid_processing
        if (RESET) begin 
            M_AXI_AWVALID <= 1'b0;
        end else begin 
            case (current_state)
                ESTABLISH_ADDR_ST : 
                    M_AXI_AWVALID <= 1'b1;

                default:
                    if (M_AXI_AWVALID & M_AXI_AWREADY) begin 
                        M_AXI_AWVALID <= 1'b0;
                    end else begin 
                        M_AXI_AWVALID <= M_AXI_AWVALID;
                    end

            endcase
        end 
    end



    always_ff @(posedge CLK) begin : M_AXI_BREADY_processing 
        if (has_bresp_flaq) begin 
            M_AXI_BREADY <= 1'b0;
        end else begin 
            M_AXI_BREADY <= 1'b1;
        end 
    end 



    always_ff @(posedge CLK) begin : m_axi_wvalid_processing
        case (current_state)

            WRITE_ST :
                if (M_AXI_WVALID & M_AXI_WLAST & M_AXI_WREADY) begin 
                    M_AXI_WVALID <= 1'b0;
                end else begin 
                    M_AXI_WVALID <= 1'b1;
                end

            default:
                M_AXI_WVALID <= 1'b0;

        endcase
    end



    always_ff @(posedge CLK) begin : awlen_counter_processing
        case (current_state)
            WRITE_ST :
                if (M_AXI_WVALID & M_AXI_WREADY) begin
                    awlen_counter <= awlen_counter + 1;
                end else begin 
                    awlen_counter <= awlen_counter;
                end

            default:
                awlen_counter <= '{default:0};
        endcase
    end



    fifo_in_sync_counted_xpm #(
        .DATA_WIDTH(DATA_WIDTH),
        .MEMTYPE   ("block"   ),
        .DEPTH     (FIFO_DEPTH)
    ) fifo_in_sync_counted_xpm_inst (
        .CLK          (CLK                          ),
        .RESET        (fifo_reset                   ),
        .S_AXIS_TDATA (S_AXIS_TDATA                 ),
        .S_AXIS_TVALID(S_AXIS_TVALID & write_ability),
        .S_AXIS_TLAST (S_AXIS_TLAST                 ),
        .S_AXIS_TREADY(S_AXIS_TREADY                ),
        .IN_DOUT_DATA (M_AXI_WDATA                  ),
        .IN_DOUT_LAST (in_dout_last                 ),
        .IN_RDEN      (in_rden                      ),
        .IN_EMPTY     (in_empty                     ),
        
        .DATA_COUNT   (fifo_data_count              )
    );



    always_comb begin : in_rden_processing 
        in_rden = M_AXI_WVALID & M_AXI_WREADY & !portion_end_flaq;
    end 



    always_comb begin  : fifo_word_count_processing 

        fifo_word_count = fifo_data_count[31:C_AXSIZE_INT];
    end 



    always_ff @(posedge CLK) begin : fifo_reset_processing
        if (RESET) begin 
            fifo_reset <= 1'b1;
        end else begin 
            case (current_state)

                IDLE_ST :
                    if (RUN | has_run_initiated | has_runned) begin 
                        fifo_reset <= 1'b0;
                    end else begin  
                        fifo_reset <= 1'b1;
                    end 

                default:
                    fifo_reset <= 1'b0;

            endcase
        end
    end



    always_ff @(posedge CLK) begin : write_ability_processing
        if (RESET | fifo_reset) begin 
            write_ability <= 1'b0;
        end else begin 

            case (current_state)
            
                IDLE_ST :
                    if (has_active_packet) begin 
                        if (has_run_initiated & S_AXIS_TVALID & S_AXIS_TLAST) begin 
                            write_ability <= 1'b1;
                        end else begin 
                            write_ability <= write_ability;
                        end 
                    end else begin 
                        if (has_runned) begin 
                            write_ability <= 1'b1;
                        end else begin 
                            write_ability <= write_ability;
                        end 
                    end 

                default:
                    write_ability <= write_ability;

            endcase
        end   
    end



    always_ff @(posedge CLK) begin : has_active_packet_processing
        if (S_AXIS_TVALID) begin 
            if (S_AXIS_TLAST) begin 
                has_active_packet <= 1'b0;
            end else begin 
                has_active_packet <= 1'b1;
            end 
        end else begin 
            has_active_packet <= has_active_packet;
        end 
    end 

    generate
        
        if (SUSPENDABLE == 0) begin : GEN_UNSUSPEND_LOGIC         

            always_ff @(posedge CLK) begin : current_state_processing
                if (RESET) begin 
                    current_state <= IDLE_ST;
                end else begin 
                    case (current_state)

                        IDLE_ST :
                            if (has_runned) begin 
                                current_state <= WAIT_FOR_DATA_ST;
                            end else begin 
                                current_state <= current_state;
                            end

                        WAIT_FOR_DATA_ST :
                            if (interval_reached) begin 
                                current_state <= IDLE_ST;
                            end else begin 
                                if (has_burst_accumulate_timer_reached | has_fifo_word_count_reached) begin 
                                    current_state <= ESTABLISH_ADDR_ST;
                                end else begin 
                                    current_state <= current_state;
                                end 
                            end 

                        ESTABLISH_ADDR_ST : 
                            current_state <= WRITE_ST;

                        WRITE_ST :
                            if (M_AXI_WVALID & M_AXI_WREADY & M_AXI_WLAST) begin
                                current_state <= WAIT_BRESP_ST;
                            end else begin 
                                current_state <= current_state;
                            end 

                        WAIT_BRESP_ST :
                            if (has_bresp_flaq) begin 
                                if ((found_last_flaq | portion_end_flaq) & word_counter_zero_flaq | interval_reached) begin 
                                    current_state <= IDLE_ST;
                                end else begin                         
                                    // if (in_empty) begin 
                                        current_state <= WAIT_FOR_DATA_ST;
                                    // end else begin 
                                        // current_state <= ESTABLISH_ADDR_ST;
                                    // end 
                                end 
                            end else begin 
                                current_state <= current_state;
                            end 

                        default:
                            current_state <= current_state;
                    endcase
                end 
            end



            always_ff @(posedge CLK) begin : TRANSMITTED_FLAQ_processing 
                case (current_state)        
                    WAIT_FOR_DATA_ST : 
                        if (interval_reached) begin 
                            TRANSMITTED_FLAQ <= 1'b1;
                        end else begin 
                            TRANSMITTED_FLAQ <= 1'b0;
                        end 

                    WAIT_BRESP_ST :
                        if (has_bresp_flaq) begin 
                            if (((found_last_flaq | portion_end_flaq) & word_counter_zero_flaq) | interval_reached) begin 
                                TRANSMITTED_FLAQ <= 1'b1;
                            end else begin                         
                                TRANSMITTED_FLAQ <= 1'b0;
                            end 
                        end else begin 
                            TRANSMITTED_FLAQ <= 1'b0;
                        end 

                    default :
                        TRANSMITTED_FLAQ <= 1'b0;
                endcase
            end  



        end // GEN_UNSUSPEND_LOGIC

        if (SUSPENDABLE == 1) begin : GEN_SUSPEND_LOGIC
            
            always_ff @(posedge CLK) begin : current_state_processing
                if (RESET) begin 
                    current_state <= IDLE_ST;
                end else begin 
                    case (current_state)

                        IDLE_ST :
                            if (has_runned) begin 
                                current_state <= WAIT_FOR_DATA_ST;
                            end else begin 
                                current_state <= current_state;
                            end

                        WAIT_FOR_DATA_ST :
                            if (interval_reached) begin 
                                if (OVERFLOW) begin 
                                    current_state <= SUSPENDED_ST;
                                end else begin 
                                    current_state <= IDLE_ST;
                                end 
                            end else begin 
                                if (has_burst_accumulate_timer_reached | has_fifo_word_count_reached) begin 
                                    current_state <= ESTABLISH_ADDR_ST;
                                end else begin 
                                    current_state <= current_state;
                                end 
                            end 

                        ESTABLISH_ADDR_ST : 
                            current_state <= WRITE_ST;

                        WRITE_ST :
                            if (M_AXI_WVALID & M_AXI_WREADY & M_AXI_WLAST) begin
                                current_state <= WAIT_BRESP_ST;
                            end else begin 
                                current_state <= current_state;
                            end 

                        WAIT_BRESP_ST :
                            if (has_bresp_flaq) begin 
                                if ((found_last_flaq | portion_end_flaq) & word_counter_zero_flaq | interval_reached) begin
                                    if (OVERFLOW) begin 
                                        current_state <= SUSPENDED_ST;
                                    end else begin 
                                        current_state <= IDLE_ST;
                                    end  
                                end else begin                         
                                    // if (in_empty) begin 
                                        current_state <= WAIT_FOR_DATA_ST;
                                    // end else begin 
                                        // current_state <= ESTABLISH_ADDR_ST;
                                    // end 
                                end 
                            end else begin 
                                current_state <= current_state;
                            end 

                        SUSPENDED_ST : 
                            if (OVERFLOW) begin 
                                current_state <= current_state;
                            end else begin 
                                current_state <= IDLE_ST;
                            end 

                        default:
                            current_state <= current_state;
                    endcase
                end 
            end


 

            always_ff @(posedge CLK) begin : TRANSMITTED_FLAQ_processing 
                case (current_state)        
                    WAIT_FOR_DATA_ST : 
                        if (interval_reached) begin 
                            TRANSMITTED_FLAQ <= 1'b1;
                        end else begin 
                            TRANSMITTED_FLAQ <= 1'b0;
                        end 

                    WAIT_BRESP_ST :
                        if (has_bresp_flaq) begin 
                            if (((found_last_flaq | portion_end_flaq) & word_counter_zero_flaq) | interval_reached) begin 
                                if (OVERFLOW) begin 
                                    TRANSMITTED_FLAQ <= 1'b0;
                                end else begin 
                                    TRANSMITTED_FLAQ <= 1'b1;
                                end 
                            end else begin                         
                                TRANSMITTED_FLAQ <= 1'b0;
                            end 
                        end else begin 
                            TRANSMITTED_FLAQ <= 1'b0;
                        end 

                    SUSPENDED_ST : 
                        if (OVERFLOW) begin 
                            TRANSMITTED_FLAQ <= 1'b0;
                        end else begin 
                            TRANSMITTED_FLAQ <= 1'b1;
                        end 

                    default :
                        TRANSMITTED_FLAQ <= 1'b0;
                endcase
            end  


        end // GEN_SUSPEND_LOGIC

    endgenerate


    always_ff @(posedge CLK) begin : unfound_last_flaq_processing 
        case (current_state) 
            WRITE_ST : 
                if (in_rden) begin 
                    if (in_dout_last) begin 
                        unfound_last_flaq <= 1'b0;
                    end else begin 
                        if (M_AXI_WLAST) begin 
                            unfound_last_flaq <= 1'b1;
                        end else begin 
                            unfound_last_flaq <= unfound_last_flaq;
                        end 
                    end 
                end else begin 
                    unfound_last_flaq <= unfound_last_flaq;
                end 
   
            default :
                unfound_last_flaq <= unfound_last_flaq;

        endcase // current_state
    end 



    always_ff @(posedge CLK) begin : found_last_flaq_processing 
        case (current_state)
            WRITE_ST :
                // if (in_rden & in_dout_last & M_AXI_WLAST) begin 
                if (in_rden & in_dout_last) begin 
                    found_last_flaq <= 1'b1;
                end else begin  
                    found_last_flaq <= 1'b0;
                end 

            WAIT_BRESP_ST : 
                if (has_bresp_flaq) begin 
                    found_last_flaq <= 1'b0;
                end else begin 
                    found_last_flaq <= found_last_flaq;
                end 

            default : 
                found_last_flaq <= 1'b0;

        endcase
    end 

    logic portion_end_flaq = 1'b0;

    always_ff @(posedge CLK) begin 
        if (word_counter_zero_flaq) begin 
            if ((unfound_last_flaq & in_dout_last & in_rden) | (found_last_flaq)) begin 
                portion_end_flaq <= 1'b1;
            end else begin 
                portion_end_flaq <= portion_end_flaq;
            end 
        end else begin 
            portion_end_flaq <= 1'b0;
        end 
    end 

    always_comb begin : has_bresp_flaq_processing

        has_bresp_flaq <= M_AXI_BVALID & M_AXI_BREADY;
    end



    always_ff @(posedge CLK) begin : burst_accumulate_timer_proc 
        if (S_AXIS_TVALID) begin 
            burst_accumulate_timer <= '{default:0};
        end else begin 
            case (current_state) 
                WAIT_FOR_DATA_ST : 
                    if (!in_empty) begin 
                        burst_accumulate_timer <= burst_accumulate_timer + 1;
                    end else begin 
                        burst_accumulate_timer <= '{default:0};
                    end 

                default : 
                    burst_accumulate_timer <= '{default:0};
            endcase // current_state
        end 
    end // current_state



    always_ff @(posedge CLK) begin : has_burst_accumulate_timer_reached_processing 
        // if (burst_accumulate_timer == (BURST_LIMIT-1)) begin 
        if (burst_accumulate_timer == (ACCUMULATE_BURST_LIMIT-1)) begin 
            
            has_burst_accumulate_timer_reached <= 1'b1;
        end else begin 
            has_burst_accumulate_timer_reached <= 1'b0;
        end 
    end 



    always_ff @(posedge CLK) begin : TRANSMITTED_BYTES_processing 
        case (current_state) 
            IDLE_ST : 
                TRANSMITTED_BYTES <= '{default:0};

            default :
                if (in_rden) begin 
                    TRANSMITTED_BYTES <= TRANSMITTED_BYTES + INCREMENT_VALUE;
                end else begin 
                    TRANSMITTED_BYTES <= TRANSMITTED_BYTES;
                end  

        endcase // current_state
    end 



    always_ff @(posedge CLK) begin : current_address_proc
        case (current_state) 
            IDLE_ST : 
                CURRENT_ADDRESS <= M_AXI_AWADDR;

            default : 
                CURRENT_ADDRESS <= CURRENT_ADDRESS;
        endcase // current_state
    end 



    always_ff @(posedge CLK) begin : bytes_to_bound_processing 
        if (M_AXI_AWADDR[12:0] == 'h1000) begin 
            bytes_to_bound <= 'h1000;
        end else begin 
            bytes_to_bound <= 13'h1000 - M_AXI_AWADDR[11:0];
        end 
    end 



    always_comb begin : words_to_bound_processing 

        words_to_bound = bytes_to_bound[12:C_AXSIZE_INT];
    end 


    always_ff @(posedge CLK) begin 
        if (RESET | TRANSMITTED_FLAQ) begin 
            interval_work <= 1'b0;
        end else begin 
            if (|TRANSMITTED_BYTES) begin 
                interval_work <= INTERVAL_USING;
            end else begin 
                interval_work <= 1'b0;
            end 
        end 
    end 

    always_ff @(posedge CLK) begin 
        if (!interval_work | RESET | TRANSMITTED_FLAQ) begin 
            interval_timer <= '{default:0};
        end else begin 
            if (interval_timer < INTERVAL) begin 
                interval_timer <= interval_timer + 1;
            end else begin 
                interval_timer <= interval_timer;
            end 
        end  
    end 

    always_ff @(posedge CLK) begin 
        if (!interval_work | TRANSMITTED_FLAQ) begin 
            interval_reached <= 1'b0;
        end else begin 
            if (interval_timer < INTERVAL) begin 
                interval_reached <= 1'b0;
            end else begin 
                interval_reached <= ~uncompleted_packet;
            end 
        end 
    end 

    always_ff @(posedge CLK) begin : uncompleted_packet_processing 
        if (RESET) begin 
            uncompleted_packet <= 1'b0;
        end else begin 
            case (current_state)
                WRITE_ST :
                    if (in_rden) begin 
                        if (in_dout_last) begin 
                            uncompleted_packet <= 1'b0;
                        end else begin 
                            uncompleted_packet <= 1'b1;
                        end 
                    end else begin 
                        uncompleted_packet <= uncompleted_packet;
                    end

                default:
                    uncompleted_packet <= uncompleted_packet;

            endcase // current_state
        end 
    end 

endmodule
