// Pluto+ block-design wrapper for the DroneID detector.
//
// RX samples and capture RAM stay in the AD9361 l_clk domain. The AXI-Lite
// register bank remains on sys_cpu_clk; control toggles and compact capture
// status are synchronized across the domains, as are Gray-coded counters.
module droneid_pluto_axi_dualrx (
    input  wire        rx_clk,
    input  wire        rx_reset,
    input  wire        sample_valid0,
    input  wire [11:0] sample_i0,
    input  wire [11:0] sample_q0,
    input  wire        sample_valid1,
    input  wire [11:0] sample_i1,
    input  wire [11:0] sample_q1,

    output wire        capture_mode,
    output wire [63:0] capture_dma_data,
    output wire        capture_dma_valid,
    input  wire        capture_dma_request,

    (* X_INTERFACE_INFO = "xilinx.com:signal:clock:1.0 s_axi_aclk CLK",
       X_INTERFACE_PARAMETER = "ASSOCIATED_BUSIF S_AXI, ASSOCIATED_RESET s_axi_aresetn, FREQ_HZ 100000000" *)
    input  wire        s_axi_aclk,
    (* X_INTERFACE_INFO = "xilinx.com:signal:reset:1.0 s_axi_aresetn RST",
       X_INTERFACE_PARAMETER = "POLARITY ACTIVE_LOW" *)
    input  wire        s_axi_aresetn,

    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 S_AXI AWADDR",
       X_INTERFACE_PARAMETER = "XIL_INTERFACENAME S_AXI, PROTOCOL AXI4LITE, DATA_WIDTH 32, ADDR_WIDTH 12, FREQ_HZ 100000000, ID_WIDTH 0, AWUSER_WIDTH 0, ARUSER_WIDTH 0, WUSER_WIDTH 0, RUSER_WIDTH 0, BUSER_WIDTH 0, READ_WRITE_MODE READ_WRITE" *)
    input  wire [11:0] s_axi_awaddr,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 S_AXI AWVALID" *)
    input  wire        s_axi_awvalid,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 S_AXI AWREADY" *)
    output wire        s_axi_awready,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 S_AXI WDATA" *)
    input  wire [31:0] s_axi_wdata,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 S_AXI WSTRB" *)
    input  wire [3:0]  s_axi_wstrb,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 S_AXI WVALID" *)
    input  wire        s_axi_wvalid,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 S_AXI WREADY" *)
    output wire        s_axi_wready,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 S_AXI BRESP" *)
    output wire [1:0]  s_axi_bresp,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 S_AXI BVALID" *)
    output reg         s_axi_bvalid,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 S_AXI BREADY" *)
    input  wire        s_axi_bready,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 S_AXI ARADDR" *)
    input  wire [11:0] s_axi_araddr,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 S_AXI ARVALID" *)
    input  wire        s_axi_arvalid,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 S_AXI ARREADY" *)
    output wire        s_axi_arready,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 S_AXI RDATA" *)
    output reg [31:0]  s_axi_rdata,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 S_AXI RRESP" *)
    output wire [1:0]  s_axi_rresp,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 S_AXI RVALID" *)
    output reg         s_axi_rvalid,
    (* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 S_AXI RREADY" *)
    input  wire        s_axi_rready
);
    assign s_axi_awready = !aw_hold && !s_axi_bvalid;
    assign s_axi_wready  = !w_hold && !s_axi_bvalid;
    assign s_axi_arready = !s_axi_rvalid;
    assign s_axi_bresp = 2'b00;
    assign s_axi_rresp = 2'b00;

    reg [11:0] awaddr_hold;
    reg [31:0] wdata_hold;
    reg [3:0] wstrb_hold;
    reg aw_hold;
    reg w_hold;
    reg enable_cpu;
    reg clear_toggle_cpu;
    reg capture_enable_cpu;
    reg software_trigger_toggle_cpu;
    reg capture_ack_toggle_cpu;

    wire [31:0] detector_count;
    wire detector_overrun;
    wire [31:0] detector_gray = detector_count ^ (detector_count >> 1);
    wire [31:0] detector_count1;
    wire [31:0] detector_gray1 = detector_count1 ^ (detector_count1 >> 1);

    (* ASYNC_REG = "TRUE" *) reg enable_meta_rx;
    (* ASYNC_REG = "TRUE" *) reg enable_sync_rx;
    (* ASYNC_REG = "TRUE" *) reg clear_meta_rx;
    (* ASYNC_REG = "TRUE" *) reg clear_sync_rx;
    reg clear_seen_rx;
    wire detector_clear = clear_sync_rx ^ clear_seen_rx;
    reg sticky_overrun_rx;

    (* ASYNC_REG = "TRUE" *) reg capture_enable_meta_rx;
    (* ASYNC_REG = "TRUE" *) reg capture_enable_sync_rx;
    (* ASYNC_REG = "TRUE" *) reg software_trigger_meta_rx;
    (* ASYNC_REG = "TRUE" *) reg software_trigger_sync_rx;
    (* ASYNC_REG = "TRUE" *) reg capture_ack_meta_rx;
    (* ASYNC_REG = "TRUE" *) reg capture_ack_sync_rx;
    reg software_trigger_seen_rx;
    reg capture_ack_seen_rx;
    wire software_trigger_pulse = software_trigger_sync_rx ^ software_trigger_seen_rx;
    wire capture_ack_pulse = capture_ack_sync_rx ^ capture_ack_seen_rx;
    wire trigger_valid0, trigger_valid1;
    wire capture_pre_ready_rx, capture_busy_rx, capture_complete_rx;
    wire [31:0] capture_trigger_sample_rx, capture_dropped_rx;
    wire [31:0] capture_dropped_gray = capture_dropped_rx ^ (capture_dropped_rx >> 1);
    wire [3:0] capture_event_count_rx;
    wire [2:0] capture_event_read_index;
    wire [31:0] capture_event_record_rx;
    (* ASYNC_REG = "TRUE" *) reg capture_pre_ready_meta_cpu, capture_pre_ready_sync_cpu;
    (* ASYNC_REG = "TRUE" *) reg capture_busy_meta_cpu, capture_busy_sync_cpu;
    (* ASYNC_REG = "TRUE" *) reg capture_complete_meta_cpu, capture_complete_sync_cpu;
    (* ASYNC_REG = "TRUE" *) reg [3:0] capture_event_count_meta_cpu, capture_event_count_sync_cpu;
    (* ASYNC_REG = "TRUE" *) reg [31:0] capture_dropped_gray_meta_cpu;
    (* ASYNC_REG = "TRUE" *) reg [31:0] capture_dropped_gray_sync_cpu;
    reg [31:0] capture_dropped_binary_cpu;

    (* ASYNC_REG = "TRUE" *) reg [31:0] count_gray_meta_cpu;
    (* ASYNC_REG = "TRUE" *) reg [31:0] count_gray_sync_cpu;
    (* ASYNC_REG = "TRUE" *) reg [31:0] count1_gray_meta_cpu;
    (* ASYNC_REG = "TRUE" *) reg [31:0] count1_gray_sync_cpu;
    (* ASYNC_REG = "TRUE" *) reg overrun_meta_cpu;
    (* ASYNC_REG = "TRUE" *) reg overrun_sync_cpu;
    reg [31:0] count_binary_cpu;
    reg [31:0] count1_binary_cpu;

    function [31:0] gray_to_binary;
        input [31:0] gray;
        integer bit_index;
        begin
            gray_to_binary[31] = gray[31];
            for (bit_index = 30; bit_index >= 0; bit_index = bit_index - 1)
                gray_to_binary[bit_index] =
                    gray_to_binary[bit_index + 1] ^ gray[bit_index];
        end
    endfunction

    wire signed [15:0] sample_i0_extended = {{4{sample_i0[11]}}, sample_i0};
    wire signed [15:0] sample_q0_extended = {{4{sample_q0[11]}}, sample_q0};
    wire signed [15:0] sample_i1_extended = {{4{sample_i1[11]}}, sample_i1};
    wire signed [15:0] sample_q1_extended = {{4{sample_q1[11]}}, sample_q1};

    droneid_fft_diagonal_dual_top detector_dualrx (
        .clk(rx_clk),
        .reset(rx_reset),
        .enable(enable_sync_rx),
        .counter_clear(detector_clear),
        .sample_valid0(sample_valid0),
        .sample_i0(sample_i0_extended),
        .sample_q0(sample_q0_extended),
        .sample_valid1(sample_valid1),
        .sample_i1(sample_i1_extended),
        .sample_q1(sample_q1_extended),
        .sample_ready0(),
        .sample_ready1(),
        .overrun(detector_overrun),
        .trigger_valid0(trigger_valid0),
        .trigger_count0(detector_count),
        .trigger_valid1(trigger_valid1),
        .trigger_count1(detector_count1)
    );

    droneid_iq_capture_dual capture_dualrx (
        .clk(rx_clk), .reset(rx_reset),
        .enable(capture_enable_sync_rx),
        .software_trigger(software_trigger_pulse),
        .capture_ack(capture_ack_pulse),
        .dma_request(capture_dma_request),
        .sample_valid0(sample_valid0), .sample_i0(sample_i0), .sample_q0(sample_q0),
        .sample_valid1(sample_valid1), .sample_i1(sample_i1), .sample_q1(sample_q1),
        .trigger0(trigger_valid0), .trigger1(trigger_valid1),
        .dma_data(capture_dma_data), .dma_valid(capture_dma_valid),
        .pretrigger_ready(capture_pre_ready_rx), .capture_busy(capture_busy_rx),
        .capture_complete(capture_complete_rx),
        .trigger_sample_index(capture_trigger_sample_rx),
        .dropped_trigger_count(capture_dropped_rx),
        .event_count(capture_event_count_rx),
        .event_read_index(capture_event_read_index),
        .event_record(capture_event_record_rx)
    );
    assign capture_mode = capture_enable_sync_rx;
    assign capture_event_read_index = s_axi_araddr[4:2];

    always @(posedge rx_clk) begin
        if (rx_reset) begin
            enable_meta_rx <= 1'b1;
            enable_sync_rx <= 1'b1;
            clear_meta_rx <= 1'b0;
            clear_sync_rx <= 1'b0;
            clear_seen_rx <= 1'b0;
            sticky_overrun_rx <= 1'b0;
            capture_enable_meta_rx <= 1'b0;
            capture_enable_sync_rx <= 1'b0;
            software_trigger_meta_rx <= 1'b0;
            software_trigger_sync_rx <= 1'b0;
            capture_ack_meta_rx <= 1'b0;
            capture_ack_sync_rx <= 1'b0;
            software_trigger_seen_rx <= 1'b0;
            capture_ack_seen_rx <= 1'b0;
        end else begin
            enable_meta_rx <= enable_cpu;
            enable_sync_rx <= enable_meta_rx;
            clear_meta_rx <= clear_toggle_cpu;
            clear_sync_rx <= clear_meta_rx;
            clear_seen_rx <= clear_sync_rx;
            capture_enable_meta_rx <= capture_enable_cpu;
            capture_enable_sync_rx <= capture_enable_meta_rx;
            software_trigger_meta_rx <= software_trigger_toggle_cpu;
            software_trigger_sync_rx <= software_trigger_meta_rx;
            software_trigger_seen_rx <= software_trigger_sync_rx;
            capture_ack_meta_rx <= capture_ack_toggle_cpu;
            capture_ack_sync_rx <= capture_ack_meta_rx;
            capture_ack_seen_rx <= capture_ack_sync_rx;
            if (detector_clear)
                sticky_overrun_rx <= 1'b0;
            else if (detector_overrun)
                sticky_overrun_rx <= 1'b1;
        end
    end

    always @(posedge s_axi_aclk) begin
        if (!s_axi_aresetn) begin
            awaddr_hold <= 12'd0;
            wdata_hold <= 32'd0;
            wstrb_hold <= 4'd0;
            aw_hold <= 1'b0;
            w_hold <= 1'b0;
            enable_cpu <= 1'b1;
            clear_toggle_cpu <= 1'b0;
            capture_enable_cpu <= 1'b0;
            software_trigger_toggle_cpu <= 1'b0;
            capture_ack_toggle_cpu <= 1'b0;
            s_axi_bvalid <= 1'b0;
            s_axi_rdata <= 32'd0;
            s_axi_rvalid <= 1'b0;
            count_gray_meta_cpu <= 32'd0;
            count_gray_sync_cpu <= 32'd0;
            count1_gray_meta_cpu <= 32'd0;
            count1_gray_sync_cpu <= 32'd0;
            overrun_meta_cpu <= 1'b0;
            overrun_sync_cpu <= 1'b0;
            count_binary_cpu <= 32'd0;
            count1_binary_cpu <= 32'd0;
            capture_pre_ready_meta_cpu <= 1'b0;
            capture_pre_ready_sync_cpu <= 1'b0;
            capture_busy_meta_cpu <= 1'b0;
            capture_busy_sync_cpu <= 1'b0;
            capture_complete_meta_cpu <= 1'b0;
            capture_complete_sync_cpu <= 1'b0;
            capture_event_count_meta_cpu <= 4'd0;
            capture_event_count_sync_cpu <= 4'd0;
            capture_dropped_gray_meta_cpu <= 32'd0;
            capture_dropped_gray_sync_cpu <= 32'd0;
            capture_dropped_binary_cpu <= 32'd0;
        end else begin
            count_gray_meta_cpu <= detector_gray;
            count_gray_sync_cpu <= count_gray_meta_cpu;
            count1_gray_meta_cpu <= detector_gray1;
            count1_gray_sync_cpu <= count1_gray_meta_cpu;
            overrun_meta_cpu <= sticky_overrun_rx;
            overrun_sync_cpu <= overrun_meta_cpu;
            count_binary_cpu <= gray_to_binary(count_gray_sync_cpu);
            count1_binary_cpu <= gray_to_binary(count1_gray_sync_cpu);
            capture_pre_ready_meta_cpu <= capture_pre_ready_rx;
            capture_pre_ready_sync_cpu <= capture_pre_ready_meta_cpu;
            capture_busy_meta_cpu <= capture_busy_rx;
            capture_busy_sync_cpu <= capture_busy_meta_cpu;
            capture_complete_meta_cpu <= capture_complete_rx;
            capture_complete_sync_cpu <= capture_complete_meta_cpu;
            capture_event_count_meta_cpu <= capture_event_count_rx;
            capture_event_count_sync_cpu <= capture_event_count_meta_cpu;
            capture_dropped_gray_meta_cpu <= capture_dropped_gray;
            capture_dropped_gray_sync_cpu <= capture_dropped_gray_meta_cpu;
            capture_dropped_binary_cpu <= gray_to_binary(capture_dropped_gray_sync_cpu);

            if (s_axi_awvalid && s_axi_awready) begin
                awaddr_hold <= s_axi_awaddr;
                aw_hold <= 1'b1;
            end
            if (s_axi_wvalid && s_axi_wready) begin
                wdata_hold <= s_axi_wdata;
                wstrb_hold <= s_axi_wstrb;
                w_hold <= 1'b1;
            end

            if (aw_hold && w_hold && !s_axi_bvalid) begin
                if ((awaddr_hold[7:0] == 8'h3c) && wstrb_hold[0]) begin
                    enable_cpu <= wdata_hold[0];
                    if (wdata_hold[1])
                        clear_toggle_cpu <= ~clear_toggle_cpu;
                end
                if ((awaddr_hold[7:0] == 8'h44) && wstrb_hold[0]) begin
                    capture_enable_cpu <= wdata_hold[0];
                    if (wdata_hold[1])
                        software_trigger_toggle_cpu <= ~software_trigger_toggle_cpu;
                    if (wdata_hold[2])
                        capture_ack_toggle_cpu <= ~capture_ack_toggle_cpu;
                end
                aw_hold <= 1'b0;
                w_hold <= 1'b0;
                s_axi_bvalid <= 1'b1;
            end else if (s_axi_bvalid && s_axi_bready) begin
                s_axi_bvalid <= 1'b0;
            end

            if (s_axi_arvalid && s_axi_arready) begin
                case (s_axi_araddr[7:0])
                    8'h38: s_axi_rdata <= count_binary_cpu;
                    8'h40: s_axi_rdata <= count1_binary_cpu;
                    8'h3c: s_axi_rdata <= {29'd0, overrun_sync_cpu, 1'b0,
                                            enable_cpu};
                    8'h44: s_axi_rdata <= {31'd0, capture_enable_cpu};
                    8'h48: s_axi_rdata <= {28'd0, capture_pre_ready_sync_cpu,
                                           capture_complete_sync_cpu,
                                           capture_busy_sync_cpu, 1'b0};
                    8'h4c: s_axi_rdata <= capture_trigger_sample_rx;
                    8'h50: s_axi_rdata <= capture_dropped_binary_cpu;
                    8'h54: s_axi_rdata <= {28'd0, capture_event_count_sync_cpu};
                    default: s_axi_rdata <= 32'd0;
                endcase
                if ((s_axi_araddr[7:0] >= 8'h80) &&
                    (s_axi_araddr[7:0] <= 8'h9c))
                    s_axi_rdata <= capture_event_record_rx;
                s_axi_rvalid <= 1'b1;
            end else if (s_axi_rvalid && s_axi_rready) begin
                s_axi_rvalid <= 1'b0;
            end
        end
    end
endmodule
