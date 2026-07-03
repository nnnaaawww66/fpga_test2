module top_sd_udp(
	input                       clk,
	input                       rst_n,

	// ===== SD卡接口 =====
	output                      sd_ncs,            // SD card chip select (SPI mode)
	output                      sd_dclk,           // SD card clock
	output                      sd_mosi,           // SD card controller data output
	input                       sd_miso,           // SD card controller data input

	// ===== 按键接口 =====
	input                       key1,

	// ===== 数码管接口 =====
	output [5:0]                seg_sel,
	output [7:0]                seg_data,

	// ===== 以太网 RGMII 接口 =====
	input                       phy1_rgmii_rx_clk,
	input                       phy1_rgmii_rx_ctl,
	input  [3:0]                phy1_rgmii_rx_data,
	output wire                 phy1_rgmii_tx_clk,
	output wire                 phy1_rgmii_tx_ctl,
	output wire [3:0]           phy1_rgmii_tx_data,

	// ===== SDRAM 接口 =====
	output                      sdram_clk
);

// ===== 参数配置 =====
parameter MEM_DATA_BITS         = 32  ;         // external memory user interface data width
parameter ADDR_BITS             = 21  ;         // external memory user interface address width
parameter BUSRT_BITS            = 10  ;         // external memory user interface burst width

// 以太网参数配置
parameter  DEVICE               = "EG4";        // FPGA型号
parameter  LOCAL_UDP_PORT_NUM   = 16'h1770;     // 本地UDP端口 6000
parameter  LOCAL_IP_ADDRESS     = 32'hc0a8f001; // 本地IP 192.168.240.1
parameter  LOCAL_MAC_ADDRESS    = 48'h0123456789ab;
parameter  DST_UDP_PORT_NUM     = 16'h1771;     // 目标UDP端口 6001
parameter  DST_IP_ADDRESS       = 32'hc0a8f002; // 目标IP 192.168.240.2

// ===== SDRAM 控制信号 =====
wire Sdr_init_done;
wire Sdr_init_ref_vld;
wire Sdr_busy;

// ===== 时钟信号 =====
wire sd_card_clk;           // SD卡时钟
wire ext_mem_clk;           // SDRAM 时钟
wire ext_mem_clk_sft;       // SDRAM 时钟偏移

// ===== 以太网相关时钟 =====
wire temac_clk;             // TEMAC 时钟 125MHz
wire udp_clk;               // UDP 协议栈时钟
wire temac_clk90;           // TEMAC 90度时钟
wire clk_125_out;
wire clk_12_5_out;
wire clk_1_25_out;
wire clk_50_out;

// ===== SD卡数据接口 =====
wire[3:0]                       state_code;
wire                            sd_card_write_en;
wire[31:0]                      sd_card_write_data;
wire                            sd_card_write_req;
wire                            sd_card_write_req_ack;
wire[6:0]                       seg_data_0;

// ===== SDRAM 读写接口 =====
wire App_rd_en;
wire [ADDR_BITS-1:0] App_rd_addr;
wire Sdr_rd_en;
wire [MEM_DATA_BITS - 1 : 0] Sdr_rd_dout;

wire App_wr_en;
wire [ADDR_BITS-1:0] App_wr_addr;
wire [MEM_DATA_BITS - 1 : 0] App_wr_din;
wire [3:0] App_wr_dm;

// ===== UDP 数据传输接口 =====
wire video_read_req;
wire video_read_req_ack;
wire video_read_en;
wire [31:0] video_read_data;

// ===== UDP 协议栈接口 =====
wire udp_tx_ready;
wire app_tx_ack;
wire app_tx_data_request;
wire app_tx_data_valid;
wire [7:0] app_tx_data;
wire [15:0] udp_data_length;

// ===== TEMAC 接口 =====
wire temac_tx_ready;
wire temac_tx_valid;
wire [7:0] temac_tx_data;
wire temac_tx_sof;
wire temac_tx_eof;

wire temac_rx_ready;
wire temac_rx_valid;
wire [7:0] temac_rx_data;
wire temac_rx_sof;
wire temac_rx_eof;

// ===== PHY 接口 =====
wire rx_clk_int;
wire rx_clk_en_int;
wire tx_clk_int;
wire tx_clk_en_int;
wire rx_valid;
wire [7:0] rx_data;
wire [7:0] tx_data;
wire tx_valid;
wire tx_rdy;
wire tx_collision;
wire tx_retransmit;
wire rx_correct_frame;
wire rx_error_frame;

// ===== 复位和配置信号 =====
wire reset;
wire reset_reg;
wire [1:0] TRI_speed;

// ===== 接收时钟 =====
wire phy1_rgmii_rx_clk_0;
wire phy1_rgmii_rx_clk_90;

assign TRI_speed = 2'b10;  // 千兆网速度
assign sdram_clk = ext_mem_clk;

// ===== TEMAC 配置信号 =====
wire tx_stop;
wire [7:0] tx_ifg_val;
wire pause_req;
wire [15:0] pause_val;
wire [47:0] pause_source_addr;
wire [47:0] unicast_address;
wire [19:0] mac_cfg_vector;

assign tx_stop = 1'b0;
assign tx_ifg_val = 8'h00;
assign pause_req = 1'b0;
assign pause_val = 16'h0;
assign pause_source_addr = 48'h5af1f2f3f4f5;

// MAC地址配置（注意字节序）
assign unicast_address = {
    LOCAL_MAC_ADDRESS[7:0],
    LOCAL_MAC_ADDRESS[15:8],
    LOCAL_MAC_ADDRESS[23:16],
    LOCAL_MAC_ADDRESS[31:24],
    LOCAL_MAC_ADDRESS[39:32],
    LOCAL_MAC_ADDRESS[47:40]
};

// MAC配置向量：地址过滤模式、流控配置、速度配置、接收器配置、发送器配置
assign mac_cfg_vector = {1'b0, 2'b00, TRI_speed, 8'b00000010, 7'b0000010};

assign reset = ~rst_n || reset_reg;

// ================================================================
// 时钟生成模块
// ================================================================

// SDRAM 时钟 PLL（生成SD卡时钟和SDRAM时钟）
sys_pll sys_pll_m0(
	.refclk                     (clk),
	.clk0_out                   (sd_card_clk),
	.clk1_out                   (ext_mem_clk),
	.clk2_out                   (ext_mem_clk_sft),
	.reset                      (1'b0)
);

// 以太网时钟生成（参考 lab_ex_4）
clk_gen_rst_gen #(
	.DEVICE(DEVICE)
) u_clk_gen (
	.reset                      (~rst_n),
	.clk_in                     (clk),
	.rst_out                    (reset_reg),
	.clk_125_out0               (temac_clk),
	.clk_125_out1               (clk_125_out),
	.clk_125_out2               (temac_clk90),
	.clk_12_5_out               (clk_12_5_out),
	.clk_1_25_out               (clk_1_25_out),
	.clk_25_out                 (clk_50_out)
);

// UDP时钟生成
udp_clk_gen #(
	.DEVICE(DEVICE)
) u_temac_clk_gen (
	.reset                      (~rst_n),
	.tri_speed                  (TRI_speed),
	.clk_125_in                 (clk_125_out),
	.clk_12_5_in                (clk_12_5_out),
	.clk_1_25_in                (clk_1_25_out),
	.udp_clk_out                (udp_clk)
);

// 接收时钟 PLL
rx_pll u_rx_pll(
	.refclk                     (phy1_rgmii_rx_clk),
	.reset                      (1'b0),
	.clk0_out                   (phy1_rgmii_rx_clk_0),
	.clk1_out                   (phy1_rgmii_rx_clk_90)
);

// ================================================================
// SD卡 BMP 文件读取模块（来自lab6）
// ================================================================

sd_card_bmp  sd_card_bmp_m0(
	.clk                        (sd_card_clk              ),
	.rst                        (~rst_n                   ),
	.key                        (key1                     ),
	.state_code                 (state_code               ),
	.bmp_width                  (16'd640                  ),  // image width
	.write_req                  (sd_card_write_req        ),
	.write_req_ack              (sd_card_write_req_ack    ),
	.write_en                   (sd_card_write_en         ),
	.write_data                 (sd_card_write_data       ),
	.SD_nCS                     (sd_ncs                   ),
	.SD_DCLK                    (sd_dclk                  ),
	.SD_MOSI                    (sd_mosi                  ),
	.SD_MISO                    (sd_miso                  )
);

// ================================================================
// 数码管显示模块（来自lab6）
// ================================================================
// state_code 说明:
// 0:SD card is initializing
// 1:wait for the button to press
// 2:looking for the BMP file
// 3:wait for the fifo
// 4:reading

seg_decoder seg_decoder_m0(
	.bin_data                   (state_code               ),
	.seg_data                   (seg_data_0               )
);

seg_scan seg_scan_m0(
	.clk                        (clk                      ),
	.rst_n                      (rst_n                    ),
	.seg_sel                    (seg_sel                  ),
	.seg_data                   (seg_data                 ),
	.seg_data_0                 ({1'b1,7'b1111_111}       ),
	.seg_data_1                 ({1'b1,7'b1111_111}       ),
	.seg_data_2                 ({1'b1,7'b1111_111}       ),
	.seg_data_3                 ({1'b1,7'b1111_111}       ),
	.seg_data_4                 ({1'b1,7'b1111_111}       ),
	.seg_data_5                 ({1'b1,seg_data_0}        )
);

// ================================================================
// UDP 摄像头控制模块（来自q4）
// ================================================================

udp_cam_ctrl u_udp_cam_ctrl (
	.clk                        (udp_clk),
	.rst_n                      (rst_n & Sdr_init_done),

	// SDRAM 读取接口
	.read_req                   (video_read_req),
	.read_req_ack               (video_read_req_ack),
	.read_en                    (video_read_en),
	.read_data                  (video_read_data),

	// UDP 发送接口
	.udp_tx_ready               (udp_tx_ready),
	.app_tx_ack                 (app_tx_ack),
	.app_tx_data_request        (app_tx_data_request),
	.app_tx_data_valid          (app_tx_data_valid),
	.app_tx_data                (app_tx_data),
	.udp_data_length            (udp_data_length)
);

// ================================================================
// SDRAM 帧缓存读写控制（来自q4，修改为SD卡写入，UDP读取）
// ================================================================

frame_read_write frame_read_write_m0(
	.mem_clk                    (ext_mem_clk),
	.rst                        (~rst_n),
	.Sdr_init_done              (Sdr_init_done),
	.Sdr_init_ref_vld           (Sdr_init_ref_vld),
	.Sdr_busy                   (Sdr_busy),

	// SDRAM 读接口
	.App_rd_en                  (App_rd_en),
	.App_rd_addr                (App_rd_addr),
	.Sdr_rd_en                  (Sdr_rd_en),
	.Sdr_rd_dout                (Sdr_rd_dout),

	// UDP读取接口（使用 udp_clk 时钟域）
	.read_clk                   (udp_clk),
	.read_req                   (video_read_req),
	.read_req_ack               (video_read_req_ack),
	.read_finish                (),
	.read_addr_0                (24'd0),
	.read_addr_1                (24'd0),
	.read_addr_2                (24'd0),
	.read_addr_3                (24'd0),
	.read_addr_index            (2'd0),
	.read_len                   (24'd307200),    // 640*480*2 (RGB565) / 2 (32bit)
	.read_en                    (video_read_en),
	.read_data                  (video_read_data),

	// SDRAM 写接口
	.App_wr_en                  (App_wr_en),
	.App_wr_addr                (App_wr_addr),
	.App_wr_din                 (App_wr_din),
	.App_wr_dm                  (App_wr_dm),

	// SD卡写入接口（使用sd_card_clk时钟域）
	.write_clk                  (sd_card_clk),
	.write_req                  (sd_card_write_req),
	.write_req_ack              (sd_card_write_req_ack),
	.write_finish               (),
	.write_addr_0               (24'd0),
	.write_addr_1               (24'd0),
	.write_addr_2               (24'd0),
	.write_addr_3               (24'd0),
	.write_addr_index           (2'd0),
	.write_len                  (24'd307200),
	.write_en                   (sd_card_write_en),
	.write_data                 (sd_card_write_data)
);

// ================================================================
// SDRAM 控制器
// ================================================================

sdram U3 (
	.Clk                        (ext_mem_clk),
	.Clk_sft                    (ext_mem_clk_sft),
	.Rst                        (~rst_n),

	.Sdr_init_done              (Sdr_init_done),
	.Sdr_init_ref_vld           (Sdr_init_ref_vld),
	.Sdr_busy                   (Sdr_busy),

	.App_wr_en                  (App_wr_en),
	.App_wr_addr                (App_wr_addr),
	.App_wr_dm                  (App_wr_dm),
	.App_wr_din                 (App_wr_din),

	.App_rd_en                  (App_rd_en),
	.App_rd_addr                (App_rd_addr),
	.Sdr_rd_en                  (Sdr_rd_en),
	.Sdr_rd_dout                (Sdr_rd_dout)
);

// ================================================================
// UDP/IP 协议栈（来自q4）
// ================================================================

udp_ip_protocol_stack #(
	.DEVICE                     (DEVICE),
	.LOCAL_UDP_PORT_NUM         (LOCAL_UDP_PORT_NUM),
	.LOCAL_IP_ADDRESS           (LOCAL_IP_ADDRESS),
	.LOCAL_MAC_ADDRESS          (LOCAL_MAC_ADDRESS)
) u3_udp_ip_protocol_stack (
	.udp_rx_clk                 (udp_clk),
	.udp_tx_clk                 (udp_clk),
	.reset                      (reset),

	// UDP 发送接口
	.udp2app_tx_ready           (udp_tx_ready),
	.udp2app_tx_ack             (app_tx_ack),
	.app_tx_request             (app_tx_data_request),
	.app_tx_data_valid          (app_tx_data_valid),
	.app_tx_data                (app_tx_data),
	.app_tx_data_length         (udp_data_length),
	.app_tx_dst_port            (DST_UDP_PORT_NUM),
	.ip_tx_dst_address          (DST_IP_ADDRESS),

	// 动态配置接口（可选）
	.input_local_udp_port_num   (LOCAL_UDP_PORT_NUM),
	.input_local_udp_port_num_valid(1'b0),
	.input_local_ip_address     (LOCAL_IP_ADDRESS),
	.input_local_ip_address_valid(1'b0),

	// UDP 接收接口（暂不使用）
	.app_rx_data_valid          (),
	.app_rx_data                (),
	.app_rx_data_length         (),
	.app_rx_port_num            (),

	// TEMAC 接口
	.temac_rx_ready             (temac_rx_ready),
	.temac_rx_valid             (!temac_rx_valid),
	.temac_rx_data              (temac_rx_data),
	.temac_rx_sof               (temac_rx_sof),
	.temac_rx_eof               (temac_rx_eof),
	.temac_tx_ready             (temac_tx_ready),
	.temac_tx_valid             (temac_tx_valid),
	.temac_tx_data              (temac_tx_data),
	.temac_tx_sof               (temac_tx_sof),
	.temac_tx_eof               (temac_tx_eof),

	.ip_rx_error                (),
	.arp_request_no_reply_error ()
);

// ================================================================
// TEMAC 模块（以太网 MAC）
// ================================================================

temac_block #(
	.DEVICE(DEVICE)
) u4_trimac_block (
	.reset                      (reset),
	.gtx_clk                    (clk_125_out),
	.gtx_clk_90                 (temac_clk90),
	.rx_clk                     (rx_clk_int),
	.rx_clk_en                  (rx_clk_en_int),
	.rx_data                    (rx_data),
	.rx_data_valid              (rx_valid),
	.rx_correct_frame           (rx_correct_frame),
	.rx_error_frame             (rx_error_frame),
	.rx_status_vector           (),
	.rx_status_vld              (),
	.tx_clk                     (tx_clk_int),
	.tx_clk_en                  (tx_clk_en_int),
	.tx_data                    (tx_data),
	.tx_data_en                 (tx_valid),
	.tx_rdy                     (tx_rdy),
	.tx_stop                    (tx_stop),
	.tx_collision               (tx_collision),
	.tx_retransmit              (tx_retransmit),
	.tx_ifg_val                 (tx_ifg_val),
	.tx_status_vector           (),
	.tx_status_vld              (),
	.pause_req                  (pause_req),
	.pause_val                  (pause_val),
	.pause_source_addr          (pause_source_addr),
	.unicast_address            (unicast_address),
	.mac_cfg_vector             (mac_cfg_vector),
	.rgmii_txd                  (phy1_rgmii_tx_data),
	.rgmii_tx_ctl               (phy1_rgmii_tx_ctl),
	.rgmii_txc                  (phy1_rgmii_tx_clk),
	.rgmii_rxd                  (phy1_rgmii_rx_data),
	.rgmii_rx_ctl               (phy1_rgmii_rx_ctl),
	.rgmii_rxc                  (phy1_rgmii_rx_clk_90),
	.inband_link_status         (),
	.inband_clock_speed         (),
	.inband_duplex_status       ()
);

// ================================================================
// 发送 FIFO
// ================================================================

tx_client_fifo #(
	.DEVICE(DEVICE)
) u6_tx_fifo (
	.rd_clk                     (tx_clk_int),
	.rd_sreset                  (reset),
	.rd_enable                  (tx_clk_en_int),
	.tx_data                    (tx_data),
	.tx_data_valid              (tx_valid),
	.tx_ack                     (tx_rdy),
	.tx_collision               (tx_collision),
	.tx_retransmit              (tx_retransmit),
	.overflow                   (),

	.wr_clk                     (udp_clk),
	.wr_sreset                  (reset),
	.wr_data                    (temac_tx_data),
	.wr_sof_n                   (temac_tx_sof),
	.wr_eof_n                   (temac_tx_eof),
	.wr_src_rdy_n               (temac_tx_valid),
	.wr_dst_rdy_n               (temac_tx_ready),
	.wr_fifo_status             ()
);

// ================================================================
// 接收 FIFO
// ================================================================

rx_client_fifo #(
	.DEVICE(DEVICE)
) u7_rx_fifo (
	.wr_clk                     (rx_clk_int),
	.wr_enable                  (rx_clk_en_int),
	.wr_sreset                  (reset),
	.rx_data                    (rx_data),
	.rx_data_valid              (rx_valid),
	.rx_good_frame              (rx_correct_frame),
	.rx_bad_frame               (rx_error_frame),
	.overflow                   (),
	.rd_clk                     (udp_clk),
	.rd_sreset                  (reset),
	.rd_data_out                (temac_rx_data),
	.rd_sof_n                   (temac_rx_sof),
	.rd_eof_n                   (temac_rx_eof),
	.rd_src_rdy_n               (temac_rx_valid),
	.rd_dst_rdy_n               (temac_rx_ready),
	.rx_fifo_status             ()
);

endmodule
