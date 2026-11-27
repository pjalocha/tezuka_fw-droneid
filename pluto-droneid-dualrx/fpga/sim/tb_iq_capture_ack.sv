`timescale 1ns/1ps

module tb_iq_capture_ack;
    reg clk = 0;
    reg reset = 1;
    reg enable = 1;
    reg software_trigger = 0;
    reg capture_ack = 0;
    reg dma_request = 0;
    reg sample_valid0 = 0;
    reg sample_valid1 = 0;
    reg signed [11:0] sample_i0 = 0;
    reg signed [11:0] sample_q0 = 0;
    reg signed [11:0] sample_i1 = 0;
    reg signed [11:0] sample_q1 = 0;
    reg trigger0 = 0;
    reg trigger1 = 0;
    wire [63:0] dma_data;
    wire dma_valid, pretrigger_ready, capture_busy, capture_complete;
    wire [31:0] trigger_sample_index, dropped_trigger_count;
    wire [3:0] event_count;
    reg [2:0] event_read_index = 0;
    wire [31:0] event_record;

    always #5 clk = ~clk;

    droneid_iq_capture_dual #(
        .PRE_SAMPLES(4), .POST_SAMPLES(4), .PTR_WIDTH(2),
        .POST_PTR_WIDTH(2), .STREAM_WIDTH(3)
    ) dut (.*);

    task push_sample;
        begin
            @(negedge clk);
            sample_valid0 = 1;
            sample_valid1 = 1;
            sample_i0 = sample_i0 + 1;
            sample_i1 = sample_i1 + 1;
            @(negedge clk);
            sample_valid0 = 0;
            sample_valid1 = 0;
        end
    endtask

    integer i;
    initial begin
        repeat (3) @(negedge clk);
        reset = 0;
        for (i = 0; i < 5; i = i + 1)
            push_sample();
        if (!pretrigger_ready)
            $fatal(1, "pretrigger history did not fill");

        @(negedge clk);
        software_trigger = 1;
        @(negedge clk);
        software_trigger = 0;
        for (i = 0; i < 4; i = i + 1)
            push_sample();
        if (!capture_complete)
            $fatal(1, "capture did not complete post-trigger window");

        // Start replay, then model the DMA stopping at its fixed transfer
        // boundary while the FPGA is in ST_DRAIN and no further request comes.
        @(negedge clk);
        dma_request = 1;
        wait (dut.state == 3'd4);
        @(negedge clk);
        dma_request = 0;
        if (!capture_busy)
            $fatal(1, "expected ST_DRAIN to remain busy before host ACK");

        @(negedge clk);
        capture_ack = 1;
        @(negedge clk);
        capture_ack = 0;
        #1;
        if (capture_busy || capture_complete || pretrigger_ready || event_count != 0)
            $fatal(1, "host ACK failed to rearm capture state");

        $display("PASS: host ACK rearms a capture stalled in ST_DRAIN");
        $finish;
    end

    initial begin
        #100000;
        $fatal(1, "capture ACK simulation timed out");
    end
endmodule
