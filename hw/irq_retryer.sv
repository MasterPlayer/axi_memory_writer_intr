`timescale 1ns / 1ps


module irq_retryer #(
    parameter integer RETRY_COUNTER_LIMIT = 250000000, 
    parameter integer CMD_FIFO_DEPTH = 64
)(
    input  logic CLK           ,
    input  logic RESET         ,
    input  logic USER_EVENT_IN ,
    input  logic USER_EVENT_ACK,
    output logic RETRY
);

    logic [$clog2(RETRY_COUNTER_LIMIT)-1:0] retry_counter      = '{default:0};
    logic [     $clog2(CMD_FIFO_DEPTH)-1:0] unload_irq_counter = '{default:0};

    logic d_user_event_in  = 1'b0;
    logic event_signal     = 1'b0;
    logic d_user_event_ack = 1'b0;
    logic event_ack_signal = 1'b0;

    logic unload_flaq = 1'b0;

    always_ff @(posedge CLK) begin : d_user_event_in_processing 
        d_user_event_in <= USER_EVENT_IN;
    end 

    always_ff @(posedge CLK) begin : event_signal_processing 
        if (USER_EVENT_IN & !d_user_event_in) begin 
            event_signal <= 1'b1;
        end else begin 
            event_signal <= 1'b0;
        end 
    end 

    always_ff @(posedge CLK) begin : d_user_event_ack_processing 
        d_user_event_ack <= USER_EVENT_ACK;
    end 

    always_ff @(posedge CLK) begin : event_ack_signal_processing 
        if (USER_EVENT_ACK & !d_user_event_ack) begin 
            event_ack_signal <= 1'b1;
        end else begin 
            event_ack_signal <= 1'b0;
        end 
    end 

    always_ff @(posedge CLK) begin : unload_irq_counter_processing 
        if (RESET) begin 
            unload_irq_counter <= '{default:0};
        end else begin 
            if (event_signal) begin 
                if (!event_ack_signal) begin 
                    if (unload_irq_counter == (CMD_FIFO_DEPTH-1)) begin 
                        unload_irq_counter <= unload_irq_counter;
                    end else begin 
                        unload_irq_counter <= unload_irq_counter + 1;
                    end 
                end else begin 
                    unload_irq_counter <= unload_irq_counter;
                end 
            end else begin 
                if (event_ack_signal) begin 
                    if (unload_irq_counter == 0) begin 
                        unload_irq_counter <= unload_irq_counter;
                    end else begin 
                        unload_irq_counter <= unload_irq_counter - 1;
                    end 
                end else begin 
                    unload_irq_counter <= unload_irq_counter;
                end 
            end 
        end 
    end 

    always_ff @(posedge CLK) begin : unload_flaq_processing 
        if (unload_irq_counter == 0) begin 
            unload_flaq <= 1'b0;
        end else begin 
            unload_flaq <= 1'b1;
        end 
    end 

    always_ff @(posedge CLK) begin : retry_counter_processing 
        if (RESET) begin 
            retry_counter <= '{default:0};
        end else begin 
            if (unload_flaq) begin 
                if (retry_counter == (RETRY_COUNTER_LIMIT-1)) begin 
                    retry_counter <= '{default:0};
                end else begin 
                    if (event_signal) begin 
                        retry_counter <= '{default:0};
                    end else begin 
                        retry_counter <= retry_counter + 1;
                    end 
                end 
            end else begin 
                retry_counter <= '{default:0};
            end 
        end 
    end 

    always_ff @(posedge CLK) begin : RETRY_processing 
        if (retry_counter == (RETRY_COUNTER_LIMIT-1)) begin 
            RETRY <= 1'b1;
        end else begin 
            RETRY <= 1'b0;
        end 
    end 

endmodule
