`timescale 1ns/1ps

module tb_fft_diagonal_dual;
    localparam integer MAX_SAMPLES = 400000;
    reg clk = 1'b0;
    always #8.138 clk = ~clk; // 61.44 MHz AD9361 l_clk domain

    reg reset = 1'b1;
    reg sample_valid0 = 1'b0;
    reg sample_valid1 = 1'b0;
    reg signed [15:0] sample_i0 = 16'sd0;
    reg signed [15:0] sample_q0 = 16'sd0;
    reg signed [15:0] sample_i1 = 16'sd0;
    reg signed [15:0] sample_q1 = 16'sd0;
    wire sample_ready0, sample_ready1, overrun;
    wire trigger_valid0, trigger_valid1;
    wire [31:0] trigger_count0, trigger_count1;
    integer samples [0:MAX_SAMPLES-1];
    integer sample_count, expected_triggers, selected_channel;
    integer observed0 = 0, observed1 = 0;
    reg replay_active = 1'b1;
    integer sample_no, timeout_cycles = 0, sample_fd, scan_status;
    reg [31:0] packed_iq;
    reg [8*512-1:0] vector_path;

    droneid_fft_diagonal_dual_top dut (
        .clk(clk), .reset(reset), .enable(1'b1), .counter_clear(1'b0),
        .sample_valid0(sample_valid0), .sample_i0(sample_i0), .sample_q0(sample_q0),
        .sample_valid1(sample_valid1), .sample_i1(sample_i1), .sample_q1(sample_q1),
        .sample_ready0(sample_ready0), .sample_ready1(sample_ready1),
        .overrun(overrun), .trigger_valid0(trigger_valid0),
        .trigger_count0(trigger_count0), .trigger_valid1(trigger_valid1),
        .trigger_count1(trigger_count1)
    );

    always @(posedge clk) begin
        timeout_cycles = timeout_cycles + 1;
        if (timeout_cycles > 5000000) begin
            $display("FAIL: dual-RX simulation watchdog expired");
            $fatal(1);
        end
        if (overrun && replay_active) begin
            $display("FAIL: shared FFT scheduler overrun at cycle %0d feed=%0d/%0d ready=%b%b sample=%0d/%0d hop=%0d/%0d tags=%0d output_ready=%b emit=%b/%0d/%0d detready=%b%b",
                     timeout_cycles, dut.fft_shared.feed_channel,
                     dut.fft_shared.feed_index, dut.fft_shared.frame_ready1,
                     dut.fft_shared.frame_ready0, dut.fft_shared.sample_count0,
                     dut.fft_shared.sample_count1, dut.fft_shared.hop_count0,
                     dut.fft_shared.hop_count1, dut.fft_shared.channel_tag_count,
                     dut.fft_shared.output_ready, dut.fft_shared.emit_active,
                     dut.fft_shared.emit_index0, dut.fft_shared.emit_index1,
                     dut.fft_shared.fft_ready1, dut.fft_shared.fft_ready0);
            $fatal(1);
        end
        if (trigger_valid0) observed0 = observed0 + 1;
        if (trigger_valid1) observed1 = observed1 + 1;
    end

    task send_sample;
        input [31:0] word;
        begin
            @(negedge clk);
            if (selected_channel == 0) begin
                sample_valid0 = 1'b1;
                sample_valid1 = 1'b1;
                sample_i0 = $signed(word[15:0]);
                sample_q0 = $signed(word[31:16]);
                sample_i1 = 16'sd0;
                sample_q1 = 16'sd0;
            end else begin
                sample_valid0 = 1'b1;
                sample_valid1 = 1'b1;
                sample_i0 = 16'sd0;
                sample_q0 = 16'sd0;
                sample_i1 = $signed(word[15:0]);
                sample_q1 = $signed(word[31:16]);
            end
            @(posedge clk);
            @(negedge clk);
            sample_valid0 = 1'b0;
            sample_valid1 = 1'b0;
            repeat (2) @(negedge clk); // 4 clocks/sample on each RX chain.
        end
    endtask

    initial begin
        if (!$value$plusargs("SAMPLES=%s", vector_path) ||
            !$value$plusargs("CHANNEL=%d", selected_channel)) begin
            $display("Usage: vvp sim.vvp +SAMPLES=file.mem +CHANNEL=0|1 [+EXPECT=count]");
            $fatal(2);
        end
        if (!$value$plusargs("EXPECT=%d", expected_triggers)) expected_triggers = 10;
        if (selected_channel != 0 && selected_channel != 1) $fatal(2);
        sample_fd = $fopen(vector_path, "r");
        if (!sample_fd) $fatal(1, "cannot open vector file %0s", vector_path);
        scan_status = $fscanf(sample_fd, "%h", sample_count);
        if (scan_status != 1 || sample_count <= 64 || sample_count > MAX_SAMPLES)
            $fatal(1, "invalid sample count %0d", sample_count);
        for (sample_no=0; sample_no<sample_count; sample_no=sample_no+1) begin
            scan_status = $fscanf(sample_fd, "%h", samples[sample_no]);
            if (scan_status != 1) $fatal(1, "short vector at sample %0d", sample_no);
        end
        $fclose(sample_fd);
        $display("Dual RX: replaying %0d samples on RX%0d; expect %0d triggers",
                 sample_count, selected_channel, expected_triggers);
        repeat (5) @(negedge clk);
        reset = 1'b0;
        for (sample_no=0; sample_no<sample_count; sample_no=sample_no+1)
            send_sample(samples[sample_no]);
        // Keep both ADC streams running while the final detector work drains.
        repeat (128) send_sample(32'd0);
        replay_active = 1'b0;
        repeat (512) @(posedge clk);
        if (selected_channel == 0) begin
            if (observed0 != expected_triggers || observed1 != 0 ||
                trigger_count0 != expected_triggers || trigger_count1 != 0)
                $fatal(1, "RX0 replay mismatch: events=%0d/%0d counters=%0d/%0d",
                       observed0, observed1, trigger_count0, trigger_count1);
        end else begin
            if (observed1 != expected_triggers || observed0 != 0 ||
                trigger_count1 != expected_triggers || trigger_count0 != 0)
                $fatal(1, "RX1 replay mismatch: events=%0d/%0d counters=%0d/%0d",
                       observed0, observed1, trigger_count0, trigger_count1);
        end
        $display("PASS: shared FFT channel tags and independent counters");
        $finish;
    end
endmodule
