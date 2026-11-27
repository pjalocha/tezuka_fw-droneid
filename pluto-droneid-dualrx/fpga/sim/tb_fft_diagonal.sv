`timescale 1ns/1ps

module tb_fft_diagonal;
    localparam integer MAX_SAMPLES = 400000;
    reg clk = 1'b0;
    always #4.069 clk = ~clk; // 122.88 MHz fabric clock

    reg reset = 1'b1;
    reg sample_valid = 1'b0;
    reg signed [15:0] sample_i = 16'sd0;
    reg signed [15:0] sample_q = 16'sd0;
    wire sample_ready;
    wire overrun;
    wire trigger_valid;
    wire [31:0] trigger_count_hw;
    reg counter_clear = 1'b0;
    reg detector_enable = 1'b1;
    wire signed [4:0] trigger_offset_bins;
    wire [31:0] trigger_start_frame;
    wire [36:0] trigger_ridge_sum;
    wire [36:0] trigger_background_sum;

    reg [31:0] samples [0:MAX_SAMPLES];
    integer sample_count;
    integer expected_triggers;
    integer trigger_count = 0;
    integer sample_no;
    integer timeout_cycles = 0;
    integer sample_fd;
    integer scan_status;
    reg [31:0] packed_iq;
    reg [8*512-1:0] vector_path;

    droneid_fft_diagonal_top dut (
        .clk(clk), .reset(reset),
        .enable(detector_enable), .counter_clear(counter_clear),
        .sample_valid(sample_valid), .sample_i(sample_i), .sample_q(sample_q),
        .sample_ready(sample_ready), .overrun(overrun),
        .trigger_valid(trigger_valid), .trigger_count(trigger_count_hw),
        .trigger_offset_bins(trigger_offset_bins),
        .trigger_start_frame(trigger_start_frame),
        .trigger_ridge_sum(trigger_ridge_sum),
        .trigger_background_sum(trigger_background_sum)
    );

    always @(posedge clk) begin
        timeout_cycles = timeout_cycles + 1;
        if (timeout_cycles > 10000000) begin
            $display("FAIL: end-to-end simulation watchdog expired");
            $fatal(1);
        end
        if (overrun) begin
            $display("FAIL: FFT frame overrun");
            $fatal(1);
        end
        if (trigger_valid) begin
            $display("TRIGGER frame=%0d offset_candidate=%0d ridge=%0d background=%0d",
                     trigger_start_frame, trigger_offset_bins,
                     trigger_ridge_sum, trigger_background_sum);
            trigger_count = trigger_count + 1;
        end
    end

    task send_sample;
        input [31:0] word;
        begin
            @(negedge clk);
            sample_valid = 1'b1;
            sample_i = $signed(word[15:0]);
            sample_q = $signed(word[31:16]);
            @(posedge clk);
            @(negedge clk);
            sample_valid = 1'b0;
            repeat (3) @(negedge clk); // 4 FPGA clocks/sample (~15.6 MS/s at 62.5 MHz).
        end
    endtask

    initial begin
        if (!$value$plusargs("SAMPLES=%s", vector_path)) begin
            $display("Usage: vvp sim.vvp +SAMPLES=file.mem [+EXPECT=count]");
            $fatal(2);
        end
        if (!$value$plusargs("EXPECT=%d", expected_triggers))
            expected_triggers = 10;

        sample_fd = $fopen(vector_path, "r");
        if (!sample_fd) begin
            $display("FAIL: cannot open sample file %0s", vector_path);
            $fatal(1);
        end
        scan_status = $fscanf(sample_fd, "%h", sample_count);
        if (scan_status != 1 || sample_count <= 64 || sample_count > MAX_SAMPLES) begin
            $display("FAIL: invalid sample count %0d", sample_count);
            $fatal(1);
        end
        for (sample_no=0; sample_no<sample_count; sample_no=sample_no+1) begin
            scan_status = $fscanf(sample_fd, "%h", samples[sample_no]);
            if (scan_status != 1) begin
                $display("FAIL: short sample file at sample %0d", sample_no);
                $fatal(1);
            end
        end
        $fclose(sample_fd);
        $display("Streaming %0d I/Q samples; expecting %0d triggers",
                 sample_count, expected_triggers);

        repeat (5) @(negedge clk);
        reset = 1'b0;
        for (sample_no=0; sample_no<sample_count; sample_no=sample_no+1)
            send_sample(samples[sample_no]);

        // Let the final completed FFT rows drain through the detector.
        repeat (256) @(posedge clk);
        if (trigger_count != expected_triggers) begin
            $display("FAIL: expected %0d triggers, got %0d",
                     expected_triggers, trigger_count);
            $fatal(1);
        end
        if (trigger_count_hw != expected_triggers) begin
            $display("FAIL: HW counter expected %0d, got %0d",
                     expected_triggers, trigger_count_hw);
            $fatal(1);
        end
        @(negedge clk);
        counter_clear = 1'b1;
        @(posedge clk);
        @(negedge clk);
        counter_clear = 1'b0;
        if (trigger_count_hw != 0) begin
            $display("FAIL: counter clear left value %0d", trigger_count_hw);
            $fatal(1);
        end
        $display("PASS: end-to-end FFT plus detector found %0d triggers",
                 trigger_count);
        $finish;
    end
endmodule
