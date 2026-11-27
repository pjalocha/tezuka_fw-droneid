// Triggered, two-RX raw-IQ capture buffer.
//
// The pre-trigger ring continuously records the four 12-bit AD936x lanes.
// A hardware or software trigger freezes that ring and starts the post buffer.
// Once the requested post samples have arrived, the completed window is
// replayed to the existing ADC DMAC source FIFO as four sign-extended S16
// lanes: RX0 I/Q followed by RX1 I/Q.
module droneid_iq_capture_dual #(
    parameter integer PRE_SAMPLES = 15360,
    parameter integer POST_SAMPLES = 15360,
    parameter integer PTR_WIDTH = $clog2(PRE_SAMPLES),
    parameter integer POST_PTR_WIDTH = $clog2(POST_SAMPLES),
    parameter integer STREAM_WIDTH = $clog2(PRE_SAMPLES + POST_SAMPLES),
    parameter integer EVENT_SLOTS = 8
) (
    input  wire               clk,
    input  wire               reset,
    input  wire               enable,
    input  wire               software_trigger,
    input  wire               capture_ack,
    input  wire               dma_request,
    input  wire               sample_valid0,
    input  wire signed [11:0] sample_i0,
    input  wire signed [11:0] sample_q0,
    input  wire               sample_valid1,
    input  wire signed [11:0] sample_i1,
    input  wire signed [11:0] sample_q1,
    input  wire               trigger0,
    input  wire               trigger1,
    output wire [63:0]        dma_data,
    output wire               dma_valid,
    output wire               pretrigger_ready,
    output wire               capture_busy,
    output wire               capture_complete,
    output reg  [31:0]        trigger_sample_index,
    output reg  [31:0]        dropped_trigger_count,
    output reg  [3:0]         event_count,
    input  wire [2:0]         event_read_index,
    output wire [31:0]        event_record
);
    localparam [2:0] ST_HISTORY = 3'd0;
    localparam [2:0] ST_POST = 3'd1;
    localparam [2:0] ST_READY = 3'd2;
    localparam [2:0] ST_STREAM = 3'd3;
    localparam [2:0] ST_DRAIN = 3'd4;

    // 4 x signed 12-bit lanes per simultaneous sample.
    (* ram_style = "block" *) reg [47:0] pre_mem [0:PRE_SAMPLES-1];
    (* ram_style = "block" *) reg [47:0] post_mem [0:POST_SAMPLES-1];
    reg [2:0] state;
    reg [PTR_WIDTH-1:0] pre_write_ptr;
    reg [PTR_WIDTH-1:0] pre_start_ptr;
    reg [PTR_WIDTH:0] pre_fill_count;
    reg [POST_PTR_WIDTH:0] post_write_count;
    reg [STREAM_WIDTH-1:0] stream_index;
    reg [47:0] pre_replay_word;
    reg [47:0] post_replay_word;
    reg replay_select_pre;
    reg replay_valid;
    reg [31:0] sample_index;
    reg [31:0] event_mem [0:EVENT_SLOTS-1];

    wire sample_pair_valid = sample_valid0 && sample_valid1;
    wire any_trigger = enable && (software_trigger || trigger0 || trigger1);
    wire [1:0] trigger_mask = {trigger1, trigger0};
    wire [1:0] trigger_source = software_trigger ? 2'd2 : 2'd1;
    wire [15:0] first_trigger_offset = sample_pair_valid ?
        (PRE_SAMPLES - 1) : PRE_SAMPLES;
    wire [47:0] packed_iq = {sample_i0, sample_q0, sample_i1, sample_q1};
    wire [STREAM_WIDTH:0] pre_sum =
        {{(STREAM_WIDTH+1-PTR_WIDTH){1'b0}}, pre_start_ptr} +
        {1'b0, stream_index};
    wire [STREAM_WIDTH:0] pre_read_addr = (pre_sum >= PRE_SAMPLES) ?
        (pre_sum - PRE_SAMPLES) : pre_sum;
    wire [STREAM_WIDTH:0] post_read_addr = stream_index - PRE_SAMPLES;
    function [15:0] sign_extend_12;
        input [11:0] value;
        begin
            sign_extend_12 = {{4{value[11]}}, value};
        end
    endfunction

    function [63:0] expand_to_iio_scan;
        input [47:0] value;
        begin
            expand_to_iio_scan = {
                sign_extend_12(value[11:0]),
                sign_extend_12(value[23:12]),
                sign_extend_12(value[35:24]),
                sign_extend_12(value[47:36])
            };
        end
    endfunction

    assign pretrigger_ready = (pre_fill_count >= PRE_SAMPLES);
    assign capture_busy = (state == ST_POST) || (state == ST_STREAM) ||
                          (state == ST_DRAIN);
    assign capture_complete = (state == ST_READY) || (state == ST_STREAM) ||
                              (state == ST_DRAIN);
    assign dma_data = expand_to_iio_scan(
        replay_select_pre ? pre_replay_word : post_replay_word);
    assign dma_valid = replay_valid;

    integer i;
    reg [PTR_WIDTH-1:0] next_pre_ptr;
    reg [1:0] event_mask;
    reg [1:0] event_source;
    reg [15:0] event_offset;
    always @(posedge clk) begin
        if (reset) begin
            state <= ST_HISTORY;
            pre_write_ptr <= {PTR_WIDTH{1'b0}};
            pre_start_ptr <= {PTR_WIDTH{1'b0}};
            pre_fill_count <= {(PTR_WIDTH+1){1'b0}};
            post_write_count <= {(POST_PTR_WIDTH+1){1'b0}};
            stream_index <= 16'd0;
            sample_index <= 32'd0;
            trigger_sample_index <= 32'd0;
            dropped_trigger_count <= 32'd0;
            event_count <= 4'd0;
            replay_valid <= 1'b0;
            replay_select_pre <= 1'b1;
            for (i = 0; i < EVENT_SLOTS; i = i + 1)
                event_mem[i] <= 32'd0;
        end else begin
            if (sample_pair_valid)
                sample_index <= sample_index + 1'b1;

            if (state == ST_HISTORY && sample_pair_valid) begin
                pre_mem[pre_write_ptr] <= packed_iq;
                if (pre_write_ptr == PRE_SAMPLES - 1)
                    pre_write_ptr <= {PTR_WIDTH{1'b0}};
                else
                    pre_write_ptr <= pre_write_ptr + 1'b1;
                if (pre_fill_count < PRE_SAMPLES)
                    pre_fill_count <= pre_fill_count + 1'b1;
            end

            if (state == ST_POST && sample_pair_valid) begin
                post_mem[post_write_count[POST_PTR_WIDTH-1:0]] <= packed_iq;
                if (post_write_count == POST_SAMPLES - 1) begin
                    post_write_count <= post_write_count + 1'b1;
                    state <= ST_READY;
                end else begin
                    post_write_count <= post_write_count + 1'b1;
                end
            end

            if (any_trigger) begin
                if (state == ST_HISTORY) begin
                    if (pre_fill_count >= PRE_SAMPLES) begin
                        // If a valid sample is being written on this edge,
                        // retain it as the final pre-trigger sample.
                        next_pre_ptr = pre_write_ptr;
                        if (sample_pair_valid) begin
                            if (pre_write_ptr == PRE_SAMPLES - 1)
                                next_pre_ptr = {PTR_WIDTH{1'b0}};
                            else
                                next_pre_ptr = pre_write_ptr + 1'b1;
                        end
                        pre_start_ptr <= next_pre_ptr;
                        trigger_sample_index <= sample_index;
                        post_write_count <= {(POST_PTR_WIDTH+1){1'b0}};
                        event_count <= 4'd1;
                        event_mask = trigger_mask;
                        if (software_trigger) event_mask = 2'b00;
                        event_mem[0] <= {12'd0, trigger_source, event_mask,
                                         first_trigger_offset};
                        state <= ST_POST;
                    end else begin
                        dropped_trigger_count <= dropped_trigger_count + 1'b1;
                    end
                end else if (state == ST_POST) begin
                    if (event_count < EVENT_SLOTS) begin
                        event_mask = trigger_mask;
                        event_source = trigger_source;
                        event_offset = PRE_SAMPLES + post_write_count;
                        event_mem[event_count] <= {12'd0, event_source,
                                                   event_mask, event_offset};
                        event_count <= event_count + 1'b1;
                    end else begin
                        dropped_trigger_count <= dropped_trigger_count + 1'b1;
                    end
                end else begin
                    dropped_trigger_count <= dropped_trigger_count + 1'b1;
                end
            end

            if (state == ST_READY && dma_request) begin
                stream_index <= 16'd0;
                replay_valid <= 1'b0;
                state <= ST_STREAM;
            end else if (state == ST_STREAM && dma_request) begin
                pre_replay_word <= pre_mem[pre_read_addr[PTR_WIDTH-1:0]];
                post_replay_word <= post_mem[post_read_addr[POST_PTR_WIDTH-1:0]];
                replay_select_pre <= (stream_index < PRE_SAMPLES);
                replay_valid <= 1'b1;
                if (stream_index == PRE_SAMPLES + POST_SAMPLES - 1) begin
                    state <= ST_DRAIN;
                end else begin
                    stream_index <= stream_index + 1'b1;
                end
            end else if (state == ST_DRAIN && dma_request) begin
                replay_valid <= 1'b0;
                state <= ST_READY;
            end

            // Linux sends the acknowledgement only after the fixed-size IIO
            // buffer has been consumed. Accept it in STREAM/DRAIN too: the
            // DMAC may stop issuing requests at the transfer boundary, which
            // otherwise leaves the replay FSM stuck in ST_DRAIN forever.
            if (capture_ack && capture_complete) begin
                state <= ST_HISTORY;
                pre_write_ptr <= {PTR_WIDTH{1'b0}};
                pre_start_ptr <= {PTR_WIDTH{1'b0}};
                pre_fill_count <= {(PTR_WIDTH+1){1'b0}};
                post_write_count <= {(POST_PTR_WIDTH+1){1'b0}};
                stream_index <= 16'd0;
                replay_valid <= 1'b0;
                event_count <= 4'd0;
            end

        end
    end
    assign event_record = event_mem[event_read_index];
endmodule
