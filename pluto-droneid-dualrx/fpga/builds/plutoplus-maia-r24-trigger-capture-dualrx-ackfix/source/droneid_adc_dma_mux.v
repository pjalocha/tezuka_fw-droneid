// Select normal live cpack samples or a frozen triggered capture for ADC DMA.
module droneid_adc_dma_mux (
    (* X_INTERFACE_INFO = "analog.com:interface:fifo_wr:1.0 live_fifo DATA",
       X_INTERFACE_PARAMETER = "XIL_INTERFACENAME live_fifo, DATA_WIDTH 64" *)
    input  wire [63:0] live_data,
    (* X_INTERFACE_INFO = "analog.com:interface:fifo_wr:1.0 live_fifo EN" *)
    input  wire        live_enable,
    (* X_INTERFACE_INFO = "analog.com:interface:fifo_wr:1.0 live_fifo OVERFLOW" *)
    output wire        live_overflow,

    input  wire        capture_mode,
    input  wire [63:0] capture_data,
    input  wire        capture_valid,

    (* X_INTERFACE_INFO = "analog.com:interface:fifo_wr:1.0 dma_fifo DATA",
       X_INTERFACE_PARAMETER = "XIL_INTERFACENAME dma_fifo, DATA_WIDTH 64" *)
    output wire [63:0] dma_data,
    (* X_INTERFACE_INFO = "analog.com:interface:fifo_wr:1.0 dma_fifo EN" *)
    output wire        dma_valid,
    (* X_INTERFACE_INFO = "analog.com:interface:fifo_wr:1.0 dma_fifo OVERFLOW" *)
    input  wire        dma_overflow,
    (* X_INTERFACE_INFO = "analog.com:interface:fifo_wr:1.0 dma_fifo XFER_REQ" *)
    input  wire        dma_xfer_req,
    output wire        dma_request
);
    assign dma_data = capture_mode ? capture_data : live_data;
    // Preserve Maia's continuous cpack behavior outside capture mode.
    assign dma_valid = capture_mode ?
                       (capture_valid && dma_xfer_req) : live_enable;
    assign live_overflow = dma_overflow;
    assign dma_request = dma_xfer_req;
endmodule
