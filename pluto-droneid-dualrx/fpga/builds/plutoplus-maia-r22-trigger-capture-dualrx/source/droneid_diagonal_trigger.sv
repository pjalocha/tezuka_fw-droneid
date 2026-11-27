// Fixed-point diagonal chirp trigger for a 64-bin FFT stream.
//
// The FFT front end presents bins in an order that makes the detector's ROI
// (shifted bins 16..47) contiguous, starting at shifted bin 13.  The detector
// stores only that 32-bin ROI, with a precomputed local-flank bit. A single
// read-port BRAM scans the center-offset diagonal; the ARM can refine
// the candidate's actual frequency offset after the trigger.
module droneid_diagonal_trigger #(
    parameter integer FFT_LOG2 = 6,
    parameter integer FFT_SIZE = 1 << FFT_LOG2,
    parameter integer HISTORY_FRAMES = 32,
    parameter integer OFFSET_TOLERANCE_BINS = 0,
    parameter integer OFFSET_CENTER_BINS = 0,
    parameter integer INPUT_WIDTH = 16,
    parameter integer COOLDOWN_FRAMES = 72
) (
    input  wire                         clk,
    input  wire                         reset,
    input  wire                         fft_valid,
    output wire                         fft_ready,
    input  wire [FFT_LOG2-1:0]          fft_bin_index,
    input  wire signed [INPUT_WIDTH-1:0] fft_i,
    input  wire signed [INPUT_WIDTH-1:0] fft_q,
    output reg                          trigger_valid,
    output reg signed [4:0]              trigger_offset_bins,
    output reg [31:0]                    trigger_start_frame,
    output reg [36:0]                    trigger_ridge_sum,
    output reg [36:0]                    trigger_background_sum
);
    localparam integer POWER_WIDTH = 2 * INPUT_WIDTH;
    localparam integer ROI_LOW = 16;
    localparam integer ROI_HIGH = 47;
    localparam integer ROI_BINS = ROI_HIGH - ROI_LOW + 1;
    localparam integer HISTORY_DEPTH = HISTORY_FRAMES * ROI_BINS;
    localparam integer HISTORY_ADDR_WIDTH = $clog2(HISTORY_DEPTH);
    localparam integer SUM_WIDTH = POWER_WIDTH + $clog2(ROI_BINS);
    localparam integer OFFSET_LANES = 2 * OFFSET_TOLERANCE_BINS + 1;
    localparam integer LANE_WIDTH = (OFFSET_LANES <= 2) ? 1 : $clog2(OFFSET_LANES);
    localparam integer ROW_WIDTH = (HISTORY_FRAMES <= 2) ? 1 : $clog2(HISTORY_FRAMES);
    localparam integer T_WIDTH = (HISTORY_FRAMES <= 2) ? 1 : $clog2(HISTORY_FRAMES);

    localparam [1:0] ST_CAPTURE = 2'd0;
    localparam [1:0] ST_SCAN = 2'd1;
    localparam [1:0] ST_DRAIN = 2'd2;
    localparam [1:0] ST_FINISH = 2'd3;

    reg [1:0] state;
    reg [ROW_WIDTH-1:0] write_row;
    reg [T_WIDTH-1:0] scan_t;
    reg [ROW_WIDTH:0] filled_rows;
    reg [31:0] total_frames;
    reg [31:0] cooldown;

    // Each lane has a separate simple-dual-port BRAM, addressed explicitly.
    reg history_write_enable;
    reg [HISTORY_ADDR_WIDTH-1:0] history_write_address;
    reg [POWER_WIDTH:0] history_write_data;
    reg [OFFSET_LANES-1:0] history_read_enable;
    reg [HISTORY_ADDR_WIDTH-1:0] history_read_address [0:OFFSET_LANES-1];
    wire [POWER_WIDTH:0] scan_data [0:OFFSET_LANES-1];
    (* ram_style = "block" *) reg [SUM_WIDTH-1:0]
        roi_history [0:HISTORY_FRAMES-1];
    reg [POWER_WIDTH-1:0] power_delay [0:5];

    reg [OFFSET_LANES-1:0] scan_lane_valid;
    reg scan_read_valid;
    reg [SUM_WIDTH-1:0] roi_scan_data;

    reg [SUM_WIDTH-1:0] ridge_accumulator [0:OFFSET_LANES-1];
    reg [SUM_WIDTH-1:0] background_accumulator [0:OFFSET_LANES-1];
    reg [T_WIDTH:0] used_bins [0:OFFSET_LANES-1];
    reg [T_WIDTH:0] flank_passes [0:OFFSET_LANES-1];
    reg [SUM_WIDTH-1:0] score_previous_previous [0:OFFSET_LANES-1];
    reg [SUM_WIDTH-1:0] score_previous [0:OFFSET_LANES-1];
    reg [OFFSET_LANES-1:0] score_previous_valid;
    reg [OFFSET_LANES-1:0] score_previous_previous_valid;
    reg [OFFSET_LANES-1:0] previous_ridge_pass;
    reg [OFFSET_LANES-1:0] previous_flank_pass;
    reg [SUM_WIDTH-1:0] roi_accumulator;

    wire signed [2*INPUT_WIDTH-1:0] i_square_signed = $signed(fft_i) * $signed(fft_i);
    wire signed [2*INPUT_WIDTH-1:0] q_square_signed = $signed(fft_q) * $signed(fft_q);
    wire [POWER_WIDTH:0] power_sum =
        {1'b0, $unsigned(i_square_signed)} + {1'b0, $unsigned(q_square_signed)};
    wire [POWER_WIDTH-1:0] bin_power = power_sum[POWER_WIDTH-1:0];
    // fft_bin_index is the stream position.  The FFT output is rotated by
    // 45 bins, so adding 13 maps it onto shifted-bin order 13..63,0..12.
    wire [FFT_LOG2-1:0] shifted_bin = fft_bin_index + 13;
    wire bin_in_roi = (shifted_bin >= ROI_LOW) && (shifted_bin <= ROI_HIGH);

    assign fft_ready = (state == ST_CAPTURE);

    function integer chirp_bin_delta;
        input integer time_index;
        integer numerator;
        integer magnitude;
        begin
            // At 15.36 MS/s, N=64, hop=32, slope=-0.135 MHz/us:
            // slope_bins_per_slice = -1.171875 = -300/256.
            numerator = -300 * (2 * time_index - (HISTORY_FRAMES - 1));
            if (numerator >= 0)
                chirp_bin_delta = (numerator + 256) / 512;
            else begin
                magnitude = -numerator;
                chirp_bin_delta = -((magnitude + 256) / 512);
            end
        end
    endfunction

    // Keep write controls and scan addresses combinational so the explicit
    // per-lane single-port memories map to BRAM instead of a large flop mux.
    always @* begin
        history_write_enable = 1'b0;
        history_write_address = {HISTORY_ADDR_WIDTH{1'b0}};
        history_write_data = {(POWER_WIDTH+1){1'b0}};
        write_center = shifted_bin - 3;
        write_power = power_delay[2];
        write_left2 = power_delay[4];
        write_right2 = power_delay[0];
        write_left3 = power_delay[5];
        write_right3 = bin_power;
        write_flank_ok = 1'b0;
        if ((state == ST_CAPTURE) && fft_valid && fft_ready &&
            (write_center >= ROI_LOW) && (write_center <= ROI_HIGH)) begin
            write_flank_ok = (write_power > write_left2) &&
                             (write_power > write_right2) &&
                             (write_power > write_left3) &&
                             (write_power > write_right3);
            history_write_enable = 1'b1;
            history_write_address = write_row * ROI_BINS +
                                    write_center - ROI_LOW;
            history_write_data = {write_flank_ok, write_power};
        end

        for (comb_lane = 0; comb_lane < OFFSET_LANES; comb_lane = comb_lane + 1) begin
            history_read_enable[comb_lane] = 1'b0;
            history_read_address[comb_lane] = {HISTORY_ADDR_WIDTH{1'b0}};
            read_lane_offset = comb_lane - OFFSET_TOLERANCE_BINS +
                               OFFSET_CENTER_BINS;
            read_predicted_bin = (FFT_SIZE / 2) + read_lane_offset +
                                 chirp_bin_delta(scan_t);
            read_row_comb = write_row + scan_t;
            if (read_row_comb >= HISTORY_FRAMES)
                read_row_comb = read_row_comb - HISTORY_FRAMES;
            if ((state == ST_SCAN) &&
                (read_predicted_bin >= ROI_LOW) &&
                (read_predicted_bin <= ROI_HIGH)) begin
                history_read_enable[comb_lane] = 1'b1;
                history_read_address[comb_lane] = read_row_comb * ROI_BINS +
                                             read_predicted_bin - ROI_LOW;
            end
        end
    end

    genvar lane_index;
    generate
        for (lane_index = 0; lane_index < OFFSET_LANES; lane_index = lane_index + 1) begin: gen_history_ram
            droneid_history_ram #(
                .DATA_WIDTH(POWER_WIDTH + 1),
                .DEPTH(HISTORY_DEPTH),
                .ADDR_WIDTH(HISTORY_ADDR_WIDTH)
            ) history_ram (
                .clk(clk),
                .write_enable(history_write_enable),
                .write_address(history_write_address),
                .write_data(history_write_data),
                .read_enable(history_read_enable[lane_index]),
                .read_address(history_read_address[lane_index]),
                .read_data(scan_data[lane_index])
            );
        end
    endgenerate

    integer lane;
    integer comb_lane;
    integer scan_row;
    integer write_center;
    integer read_row_comb;
    integer read_lane_offset;
    integer read_predicted_bin;
    reg [POWER_WIDTH-1:0] write_power;
    reg [POWER_WIDTH-1:0] write_left2;
    reg [POWER_WIDTH-1:0] write_right2;
    reg [POWER_WIDTH-1:0] write_left3;
    reg [POWER_WIDTH-1:0] write_right3;
    reg write_flank_ok;
    reg [POWER_WIDTH-1:0] p;
    reg [POWER_WIDTH-1:0] left2;
    reg [POWER_WIDTH-1:0] right2;
    reg [POWER_WIDTH-1:0] left3;
    reg [POWER_WIDTH-1:0] right3;
    reg flank_ok;
    reg [SUM_WIDTH-1:0] roi_final;
    reg [SUM_WIDTH+1:0] background_x4;
    reg [SUM_WIDTH+1:0] ridge_extended;
    reg [SUM_WIDTH+6:0] peak_x100;
    reg [SUM_WIDTH+6:0] left_x105;
    reg [SUM_WIDTH+6:0] right_x105;
    reg found_qualifying_lane;

    always @(posedge clk) begin
        if (reset) begin
            state <= ST_CAPTURE;
            write_row <= {ROW_WIDTH{1'b0}};
            scan_t <= {T_WIDTH{1'b0}};
            filled_rows <= {(ROW_WIDTH+1){1'b0}};
            total_frames <= 32'd0;
            cooldown <= 32'd0;
            roi_accumulator <= {SUM_WIDTH{1'b0}};
            scan_lane_valid <= {OFFSET_LANES{1'b0}};
            scan_read_valid <= 1'b0;
            roi_scan_data <= {SUM_WIDTH{1'b0}};
            score_previous_valid <= {OFFSET_LANES{1'b0}};
            score_previous_previous_valid <= {OFFSET_LANES{1'b0}};
            previous_ridge_pass <= {OFFSET_LANES{1'b0}};
            previous_flank_pass <= {OFFSET_LANES{1'b0}};
            trigger_valid <= 1'b0;
            trigger_offset_bins <= 5'sd0;
            trigger_start_frame <= 32'd0;
            trigger_ridge_sum <= {SUM_WIDTH{1'b0}};
            trigger_background_sum <= {SUM_WIDTH{1'b0}};
            for (lane = 0; lane < OFFSET_LANES; lane = lane + 1) begin
                ridge_accumulator[lane] <= {SUM_WIDTH{1'b0}};
                background_accumulator[lane] <= {SUM_WIDTH{1'b0}};
                used_bins[lane] <= {(T_WIDTH+1){1'b0}};
                flank_passes[lane] <= {(T_WIDTH+1){1'b0}};
                score_previous_previous[lane] <= {SUM_WIDTH{1'b0}};
                score_previous[lane] <= {SUM_WIDTH{1'b0}};
            end
            for (lane = 0; lane < 6; lane = lane + 1)
                power_delay[lane] <= {POWER_WIDTH{1'b0}};
        end else begin
            trigger_valid <= 1'b0;

            case (state)
                ST_CAPTURE: begin
                    if (fft_valid && fft_ready) begin
                        if (bin_in_roi)
                            roi_accumulator <= roi_accumulator + bin_power;

                        if (fft_bin_index == FFT_SIZE - 1) begin
                            roi_final = roi_accumulator;
                            if (bin_in_roi)
                                roi_final = roi_accumulator + bin_power;
                            roi_history[write_row] <= roi_final;
                            roi_accumulator <= {SUM_WIDTH{1'b0}};
                            total_frames <= total_frames + 1'b1;
                            if (write_row == HISTORY_FRAMES - 1)
                                write_row <= {ROW_WIDTH{1'b0}};
                            else
                                write_row <= write_row + 1'b1;

                            if (filled_rows < HISTORY_FRAMES)
                                filled_rows <= filled_rows + 1'b1;
                            if (filled_rows >= HISTORY_FRAMES - 1) begin
                                for (lane = 0; lane < OFFSET_LANES; lane = lane + 1) begin
                                    ridge_accumulator[lane] <= {SUM_WIDTH{1'b0}};
                                    background_accumulator[lane] <= {SUM_WIDTH{1'b0}};
                                    used_bins[lane] <= {(T_WIDTH+1){1'b0}};
                                    flank_passes[lane] <= {(T_WIDTH+1){1'b0}};
                                end
                                scan_t <= {T_WIDTH{1'b0}};
                                scan_read_valid <= 1'b0;
                                state <= ST_SCAN;
                            end
                        end

                        for (lane = 5; lane > 0; lane = lane - 1)
                            power_delay[lane] <= power_delay[lane-1];
                        power_delay[0] <= bin_power;
                    end
                end

                ST_SCAN: begin
                    // Synchronous BRAM read.  Issue the next row while the
                    // previous row's power/flank values are accumulated.
                    if (scan_read_valid) begin
                        for (lane = 0; lane < OFFSET_LANES; lane = lane + 1) begin
                            if (scan_lane_valid[lane]) begin
                                p = scan_data[lane][POWER_WIDTH-1:0];
                                ridge_accumulator[lane] <= ridge_accumulator[lane] + p;
                                background_accumulator[lane] <=
                                    background_accumulator[lane] + (roi_scan_data >> 5);
                                used_bins[lane] <= used_bins[lane] + 1'b1;
                                if (scan_data[lane][POWER_WIDTH])
                                    flank_passes[lane] <= flank_passes[lane] + 1'b1;
                            end
                        end
                    end

                    scan_row = write_row + scan_t;
                    if (scan_row >= HISTORY_FRAMES)
                        scan_row = scan_row - HISTORY_FRAMES;
                    roi_scan_data <= roi_history[scan_row];
                    for (lane = 0; lane < OFFSET_LANES; lane = lane + 1) begin
                        scan_lane_valid[lane] <= history_read_enable[lane];
                    end
                    scan_read_valid <= 1'b1;
                    if (scan_t == HISTORY_FRAMES - 1)
                        state <= ST_DRAIN;
                    else
                        scan_t <= scan_t + 1'b1;
                end

                ST_DRAIN: begin
                    if (scan_read_valid) begin
                        for (lane = 0; lane < OFFSET_LANES; lane = lane + 1) begin
                            if (scan_lane_valid[lane]) begin
                                p = scan_data[lane][POWER_WIDTH-1:0];
                                ridge_accumulator[lane] <= ridge_accumulator[lane] + p;
                                background_accumulator[lane] <=
                                    background_accumulator[lane] + (roi_scan_data >> 5);
                                used_bins[lane] <= used_bins[lane] + 1'b1;
                                if (scan_data[lane][POWER_WIDTH])
                                    flank_passes[lane] <= flank_passes[lane] + 1'b1;
                            end
                        end
                    end
                    scan_read_valid <= 1'b0;
                    state <= ST_FINISH;
                end

                ST_FINISH: begin
                    found_qualifying_lane = 1'b0;
                    for (lane = 0; lane < OFFSET_LANES; lane = lane + 1) begin
                        background_x4 = {background_accumulator[lane], 2'b00};
                        ridge_extended = {{2{1'b0}}, ridge_accumulator[lane]};

                        if (!found_qualifying_lane && cooldown == 0 &&
                            score_previous_valid[lane] &&
                            score_previous_previous_valid[lane] &&
                            (used_bins[lane] >= HISTORY_FRAMES / 2) &&
                            (background_accumulator[lane] != 0) &&
                            previous_ridge_pass[lane] && previous_flank_pass[lane] &&
                            (score_previous[lane] > score_previous_previous[lane]) &&
                            (score_previous[lane] > ridge_accumulator[lane])) begin
                            peak_x100 = ({7'b0, score_previous[lane]} << 6) +
                                        ({7'b0, score_previous[lane]} << 5) +
                                        ({7'b0, score_previous[lane]} << 2);
                            left_x105 = ({7'b0, score_previous_previous[lane]} << 6) +
                                        ({7'b0, score_previous_previous[lane]} << 5) +
                                        ({7'b0, score_previous_previous[lane]} << 2) +
                                        ({7'b0, score_previous_previous[lane]} << 2) +
                                         {7'b0, score_previous_previous[lane]};
                            right_x105 = ({7'b0, ridge_accumulator[lane]} << 6) +
                                         ({7'b0, ridge_accumulator[lane]} << 5) +
                                         ({7'b0, ridge_accumulator[lane]} << 2) +
                                         ({7'b0, ridge_accumulator[lane]} << 2) +
                                          {7'b0, ridge_accumulator[lane]};

                            if ((peak_x100 >= left_x105) &&
                                (peak_x100 >= right_x105)) begin
                                trigger_valid <= 1'b1;
                                trigger_offset_bins <= lane - OFFSET_TOLERANCE_BINS +
                                                       OFFSET_CENTER_BINS;
                                trigger_start_frame <= total_frames - HISTORY_FRAMES - 1;
                                trigger_ridge_sum <= score_previous[lane];
                                trigger_background_sum <= background_accumulator[lane];
                                found_qualifying_lane = 1'b1;
                                cooldown <= COOLDOWN_FRAMES;
                            end
                        end

                        score_previous_previous[lane] <= score_previous[lane];
                        score_previous[lane] <= ridge_accumulator[lane];
                        score_previous_previous_valid[lane] <= score_previous_valid[lane];
                        score_previous_valid[lane] <=
                            (used_bins[lane] >= HISTORY_FRAMES / 2) &&
                            (background_accumulator[lane] != 0);
                        previous_ridge_pass[lane] <=
                            (ridge_extended >= background_x4);
                        previous_flank_pass[lane] <=
                            (flank_passes[lane] * 5 >= used_bins[lane] * 4);
                    end

                    if ((cooldown != 0) && !found_qualifying_lane)
                        cooldown <= cooldown - 1'b1;
                    state <= ST_CAPTURE;
                end

                default: state <= ST_CAPTURE;
            endcase
        end
    end
endmodule

module droneid_history_ram #(
    parameter integer DATA_WIDTH = 33,
    parameter integer DEPTH = 1024,
    parameter integer ADDR_WIDTH = 10
) (
    input  wire                  clk,
    input  wire                  write_enable,
    input  wire [ADDR_WIDTH-1:0] write_address,
    input  wire [DATA_WIDTH-1:0] write_data,
    input  wire                  read_enable,
    input  wire [ADDR_WIDTH-1:0] read_address,
    output reg  [DATA_WIDTH-1:0] read_data
);
    (* ram_style = "block" *) reg [DATA_WIDTH-1:0] memory [0:DEPTH-1];

    always @(posedge clk) begin
        if (write_enable)
            memory[write_address] <= write_data;
        if (read_enable)
            read_data <= memory[read_address];
    end
endmodule
