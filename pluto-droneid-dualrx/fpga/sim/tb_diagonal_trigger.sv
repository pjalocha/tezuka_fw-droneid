`timescale 1ns/1ps

module tb_diagonal_trigger;
    localparam integer MAX_WORDS = 500000;

    reg clk = 1'b0;
    always #8 clk = ~clk; // 62.5 MHz design clock

    reg reset = 1'b1;
    reg fft_valid = 1'b0;
    wire fft_ready;
    reg [5:0] fft_bin_index = 6'd0;
    reg signed [15:0] fft_i = 16'sd0;
    reg signed [15:0] fft_q = 16'sd0;

    wire trigger_valid;
    wire signed [4:0] trigger_offset_bins;
    wire [31:0] trigger_start_frame;
    wire [36:0] trigger_ridge_sum;
    wire [36:0] trigger_background_sum;

    reg [31:0] vectors [0:MAX_WORDS-1];
    integer frame_count;
    integer expected_triggers;
    integer trigger_count = 0;
    integer frame;
    integer bin;
    integer word_index;
    integer spectrum_bin;
    integer timeout_cycles = 0;
    integer vector_fd;
    integer scan_status;
    reg [31:0] packed_iq;
    reg [8*512-1:0] vector_path;

    droneid_diagonal_trigger dut (
        .clk(clk),
        .reset(reset),
        .fft_valid(fft_valid),
        .fft_ready(fft_ready),
        .fft_bin_index(fft_bin_index),
        .fft_i(fft_i),
        .fft_q(fft_q),
        .trigger_valid(trigger_valid),
        .trigger_offset_bins(trigger_offset_bins),
        .trigger_start_frame(trigger_start_frame),
        .trigger_ridge_sum(trigger_ridge_sum),
        .trigger_background_sum(trigger_background_sum)
    );

    always @(posedge clk) begin
        if (trigger_valid) begin
            $display("TRIGGER frame=%0d offset_bins=%0d ridge_sum=%0d background_sum=%0d",
                     trigger_start_frame, trigger_offset_bins,
                     trigger_ridge_sum, trigger_background_sum);
            trigger_count = trigger_count + 1;
        end
        timeout_cycles = timeout_cycles + 1;
        if (timeout_cycles > 2000000) begin
            $display("FAIL: simulation watchdog expired");
            $fatal(1);
        end
    end

    task send_bin;
        input [5:0] index;
        input [31:0] word;
        begin
            @(negedge clk);
            fft_valid = 1'b1;
            fft_bin_index = index;
            fft_i = $signed(word[15:0]);
            fft_q = $signed(word[31:16]);
            while (!fft_ready) @(negedge clk);
            @(posedge clk);
            @(negedge clk);
            fft_valid = 1'b0;
        end
    endtask

    initial begin
        if (!$value$plusargs("VECTORS=%s", vector_path)) begin
            $display("Usage: vvp sim.vvp +VECTORS=file.mem [+EXPECT=count]");
            $fatal(2);
        end
        if (!$value$plusargs("EXPECT=%d", expected_triggers))
            expected_triggers = 10;

        vector_fd = $fopen(vector_path, "r");
        if (!vector_fd) begin
            $display("FAIL: cannot open vector file %0s", vector_path);
            $fatal(1);
        end
        scan_status = $fscanf(vector_fd, "%h", frame_count);
        if (scan_status != 1 || frame_count <= 0 ||
            (frame_count * 64) > MAX_WORDS) begin
            $display("FAIL: invalid vector frame count %0d", frame_count);
            $fatal(1);
        end
        for (word_index=0; word_index<frame_count*64; word_index=word_index+1) begin
            scan_status = $fscanf(vector_fd, "%h", vectors[word_index]);
            if (scan_status != 1) begin
                $display("FAIL: short vector file at word %0d", word_index);
                $fatal(1);
            end
        end
        $fclose(vector_fd);
        $display("Running %0d FFT frames; expecting %0d triggers",
                 frame_count, expected_triggers);

        repeat (5) @(negedge clk);
        reset = 1'b0;
        word_index = 0;
        for (frame = 0; frame < frame_count; frame = frame + 1) begin
            for (bin = 0; bin < 64; bin = bin + 1) begin
                // The detector input order starts at natural FFT bin 45.
                spectrum_bin = (bin + 45) & 63;
                packed_iq = vectors[word_index + spectrum_bin];
                send_bin(bin[5:0], packed_iq);
            end
            word_index = word_index + 64;
        end

        repeat (100) @(posedge clk);
        if (trigger_count != expected_triggers) begin
            $display("FAIL: expected %0d triggers, got %0d",
                     expected_triggers, trigger_count);
            $fatal(1);
        end
        $display("PASS: %0d triggers from %0d frames", trigger_count, frame_count);
        $finish;
    end
endmodule
