`timescale 1ns/1ps

module tb_droneid_pluto_axi;
    reg rx_clk = 0;
    reg rx_reset = 1;
    reg sample_valid = 0;
    reg [11:0] sample_i = 0;
    reg [11:0] sample_q = 0;
    reg s_axi_aclk = 0;
    reg s_axi_aresetn = 0;
    reg [11:0] s_axi_awaddr = 0;
    reg s_axi_awvalid = 0;
    wire s_axi_awready;
    reg [31:0] s_axi_wdata = 0;
    reg [3:0] s_axi_wstrb = 4'hf;
    reg s_axi_wvalid = 0;
    wire s_axi_wready;
    wire [1:0] s_axi_bresp;
    wire s_axi_bvalid;
    reg s_axi_bready = 0;
    reg [11:0] s_axi_araddr = 0;
    reg s_axi_arvalid = 0;
    wire s_axi_arready;
    wire [31:0] s_axi_rdata;
    wire [1:0] s_axi_rresp;
    wire s_axi_rvalid;
    reg s_axi_rready = 0;

    always #8.138 rx_clk = ~rx_clk;
    always #5 s_axi_aclk = ~s_axi_aclk;

    droneid_pluto_axi_maia dut (.*);

    task axi_write;
        input [11:0] address;
        input [31:0] value;
        begin
            @(negedge s_axi_aclk);
            s_axi_awaddr = address;
            s_axi_wdata = value;
            s_axi_awvalid = 1;
            s_axi_wvalid = 1;
            while (!(s_axi_awready && s_axi_wready))
                @(negedge s_axi_aclk);
            @(negedge s_axi_aclk);
            s_axi_awvalid = 0;
            s_axi_wvalid = 0;
            while (!s_axi_bvalid)
                @(negedge s_axi_aclk);
            s_axi_bready = 1;
            @(negedge s_axi_aclk);
            s_axi_bready = 0;
        end
    endtask

    task axi_read;
        input [11:0] address;
        output reg [31:0] result;
        begin
            @(negedge s_axi_aclk);
            s_axi_araddr = address;
            s_axi_arvalid = 1;
            while (!s_axi_arready)
                @(negedge s_axi_aclk);
            @(negedge s_axi_aclk);
            s_axi_arvalid = 0;
            while (!s_axi_rvalid)
                @(negedge s_axi_aclk);
            result = s_axi_rdata;
            s_axi_rready = 1;
            @(negedge s_axi_aclk);
            s_axi_rready = 0;
        end
    endtask

    reg [31:0] value;
    initial begin
        repeat (5) @(negedge rx_clk);
        rx_reset = 0;
        repeat (5) @(negedge s_axi_aclk);
        s_axi_aresetn = 1;

        axi_read(12'h03c, value);
        if (value[0] !== 1'b1 || value[2] !== 1'b0)
            $fatal(1, "default control register wrong: %08x", value);

        axi_write(12'h03c, 32'h00000000);
        repeat (4) @(negedge rx_clk);
        axi_read(12'h03c, value);
        if (value[0] !== 1'b0)
            $fatal(1, "enable did not clear: %08x", value);

        axi_write(12'h03c, 32'h00000003);
        repeat (4) @(negedge rx_clk);
        axi_read(12'h03c, value);
        if (value[0] !== 1'b1 || value[2] !== 1'b0)
            $fatal(1, "enable/clear control wrong: %08x", value);

        axi_read(12'h038, value);
        if (value !== 32'd0)
            $fatal(1, "unexpected RX0 counter value: %08x", value);
        $display("PASS: AXI-Lite control and counter register map");
        $finish;
    end

    initial begin
        #100000;
        $fatal(1, "AXI-Lite wrapper test timed out");
    end
endmodule
