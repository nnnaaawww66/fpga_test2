module top_integrated(
	input                       clk,
	input                       rst_n,

	// ===== SD卡接口 =====
	output                      sd_ncs,            // SD card chip select (SPI mode)
	output                      sd_dclk,           // SD card clock
	output                      sd_mosi,           // SD card controller data output
	input                       sd_miso,           // SD card controller data input

	// ===== 按键接口 =====
	input                       key1,              // SD卡读取触发按键
	input                       key2,              // 预留按键
    input       [3:0]           dip_switch,        // 拨码开关1-4对应功能1-4

	// ===== 数码管接口 =====
	//output [5:0]                seg_sel,
	//output [7:0]                seg_data,
    output            [3:0] led_data ,    
    output            [15:0] dled,

	// ===== 摄像头接口 =====
	input                       cam_pclk,          // cmos 数据像素时钟
	input                       cam_vsync,         // cmos 场同步信号
	input                       cam_href,          // cmos 行同步信号
	input   [7:0]               cam_data,          // cmos 数据
	output                      cam_rst_n,         // cmos 复位信号，低电平有效
	output                      cam_pwdn,          // 电源休眠模式选择 0：正常模式 1：电源休眠模式
	output                      cam_scl,           // cmos SCCB_SCL线
	inout                       cam_sda,           // cmos SCCB_SDA线

	// ===== 以太网 RGMII 接口 =====
	input                       phy1_rgmii_rx_clk,
	input                       phy1_rgmii_rx_ctl,
	input  [3:0]                phy1_rgmii_rx_data,
	output wire                 phy1_rgmii_tx_clk,
	output wire                 phy1_rgmii_tx_ctl,
	output wire [3:0]           phy1_rgmii_tx_data,

	// ===== SDRAM 接口 =====
	output                      sdram_clk,
    
    
    output			HDMI_CLK_P,
	output			HDMI_D2_P,
	output			HDMI_D1_P,
	output			HDMI_D0_P
);

// ===== 参数配置 =====
parameter MEM_DATA_BITS         = 32  ;         // external memory user interface data width
parameter ADDR_BITS             = 21  ;         // external memory user interface address width
parameter BUSRT_BITS            = 10  ;         // external memory user interface burst width

// 以太网参数配置
parameter  DEVICE               = "EG4";        // FPGA型号
parameter  LOCAL_UDP_PORT_NUM   = 16'h1773;     // 本地UDP端口 6003
parameter  LOCAL_IP_ADDRESS     = 32'hc0a8f001; // 本地IP 192.168.240.1
parameter  LOCAL_MAC_ADDRESS    = 48'h0123456789ab;
parameter  DST_UDP_PORT_NUM     = 16'h1773;     // 目标UDP端口 6003
parameter  DST_IP_ADDRESS       = 32'hc0a8f002; // 目标IP 192.168.240.2

// CMOS 分辨率参数
parameter  V_CMOS_DISP = 11'd480;               // CMOS分辨率--行
parameter  H_CMOS_DISP = 11'd640;               // CMOS分辨率--列
parameter  TOTAL_H_PIXEL = H_CMOS_DISP + 12'd1216; // CMOS总像素--行
parameter  TOTAL_V_PIXEL = V_CMOS_DISP + 12'd504;  // CMOS总像素--列

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

// ===== 接收时钟 =====
wire phy1_rgmii_rx_clk_0;
wire phy1_rgmii_rx_clk_90;

wire         app_rx_data_valid; 
wire [7:0]   app_rx_data;       
wire [15:0]  app_rx_data_length;
wire [15:0]  app_rx_port_num;

// ===== SD卡数据接口 =====
wire[3:0]                       state_code;
wire                            sd_card_write_en;
wire[31:0]                      sd_card_write_data;
wire                            sd_card_write_req;
wire                            sd_card_write_req_ack;
wire[6:0]                       seg_data_0;

// ===== 摄像头数据接口 =====
wire cmos_frame_vsync;
wire cmos_frame_href;
wire cmos_frame_valid;
wire [15:0] cmos_wr_data;

wire cam_write_en;
wire [31:0] cam_write_data;
wire cam_write_req;
wire cam_write_req_ack;

// ===== UDP 数据接口（功能3） =====
wire udp_write_en;
wire [31:0] udp_write_data;
wire udp_write_req;
wire udp_write_req_ack;
assign udp_write_en = 1'b0;  // 暂未实现
assign udp_write_data = 32'h00000000; // 暂未实现
assign udp_write_req = 1'b0; // 暂未实现

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

// ===== 模式选择信号 =====
reg [1:0] current_function;     // 当前功能(00:功能1, 01:功能2, 10:功能3, 11:功能4)
// 拨码开关功能选择逻辑
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        current_function <= 2'd0; // 默认功能1
    end else begin
        case(dip_switch)
            4'b0001: current_function <= 2'd0; // 拨码1对应功能1
            4'b0010: current_function <= 2'd1; // 拨码2对应功能2
            4'b0100: current_function <= 2'd2; // 拨码3对应功能3
            4'b1000: current_function <= 2'd3; // 拨码4对应功能4
            default: current_function <= 2'd0; // 默认功能1
        endcase
    end
end
wire write_clk_sel;
wire write_req_sel;
wire write_req_ack_sel;
wire write_en_sel;
wire [31:0] write_data_sel;

// ===== TEMAC 配置信号 =====
wire tx_stop;
wire [7:0] tx_ifg_val;
wire pause_req;
wire [15:0] pause_val;
wire [47:0] pause_source_addr;
wire [47:0] unicast_address;
wire [19:0] mac_cfg_vector;

assign TRI_speed = 2'b10;  // 千兆网速度
assign sdram_clk = ext_mem_clk;

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

// ===== 功能LED指示 =====
reg [7:0] mode_led;
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        mode_led <= 8'b00000001; // 默认功能1 LED0亮
    end else begin
        case(current_function)
            2'd0: mode_led <= 8'b00000001; // 功能1: LED0亮
            2'd1: mode_led <= 8'b00000010; // 功能2: LED1亮
            2'd2: mode_led <= 8'b00000100; // 功能3: LED2亮
            2'd3: mode_led <= 8'b00001000; // 功能4: LED3亮
        endcase
    end
end
assign led = mode_led;

// ================================================================
// 模式选择逻辑（按键防抖和模式切换）
// ================================================================



// ================================================================
// 数据源多路选择器
// ================================================================

// 功能数据源多路选择器（4功能）
assign write_clk_sel = (current_function == 2'd0) ? sd_card_clk :  // 功能1: SD卡
                       (current_function == 2'd1) ? cam_pclk :     // 功能2: 摄像头
                       (current_function == 2'd2) ? udp_clk :      // 功能3: UDP
                       sd_card_clk; // 功能4: 预留

assign write_req_sel = (current_function == 2'd0) ? sd_card_write_req :
                       (current_function == 2'd1) ? cam_write_req :
                       (current_function == 2'd2) ? udp_write_req : // UDP写请求
                       1'b0; // 功能4预留

assign write_en_sel = (current_function == 2'd0) ? sd_card_write_en :
                      (current_function == 2'd1) ? cam_write_en :
                      (current_function == 2'd2) ? udp_write_en :  // UDP写使能
                      1'b0; // 功能4预留

assign write_data_sel = (current_function == 2'd0) ? sd_card_write_data :
                        (current_function == 2'd1) ? cam_write_data :
                        (current_function == 2'd2) ? udp_write_data : // UDP写数据
                        32'h00000000; // 功能4预留

// 请求应答信号分配
assign sd_card_write_req_ack = (current_function == 2'd0) ? write_req_ack_sel : 1'b0;
assign cam_write_req_ack = (current_function == 2'd1) ? write_req_ack_sel : 1'b0;
assign udp_write_req_ack = (current_function == 2'd2) ? write_req_ack_sel : 1'b0; // UDP应答
assign reserved_write_req_ack = (current_function == 2'd3) ? write_req_ack_sel : 1'b0;

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

// 以太网时钟生成
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


//-----------------------------------------------------

clk_wize u0_clk_wize (
  .refclk(clk),
  .reset(1'b0),
  .clk0_out(pixel_clk_5x),
  .clk1_out(pixel_clk   )
);
// // -----------------------------------------------------
wire VGA_EN;
wire  dis_en;
wire [23:0] VGA_D;
// // app
app u1_app (
    .sys_clk                    (pixel_clk                ),
    .udp_rx_clk                 (udp_clk                ),
    .udp_tx_clk                 (udp_clk                ),
    .reset                      (key2                  ), 
    .app_rx_data_valid          (app_rx_data_valid      ), 
    .app_rx_data                (app_rx_data            ), 
    .app_rx_data_length         (app_rx_data_length     ), 
    .app_rx_port_num            (app_rx_port_num        ),
    .VGA_HSYNC	                (VGA_HSYNC              ),
	.VGA_VSYNC 	                (VGA_VSYNC              ),
	.VGA_D                      (VGA_D                  ),
    .rd_en                      (rd_en                  ),
    .VGA_EN                     (VGA_EN)
);      

    	wire [7:0]	VGA_R;
		wire [7:0]	VGA_G;
		wire [7:0]	VGA_B;

assign VGA_R = VGA_D[23:16];	
assign VGA_G = VGA_D[15:8];
assign VGA_B = VGA_D[7:0];
hdmi_tx #(.FAMILY("EG4"))	//EF2、EF3、EG4、AL3、PH1

 u2_hdmi_tx
	(
		.PXLCLK_I(pixel_clk),
		.PXLCLK_5X_I(pixel_clk_5x),

		.RST_N (key2),
		
		//VGA
		.VGA_HS (VGA_HSYNC ),
		.VGA_VS (VGA_VSYNC ),
		.VGA_DE (VGA_EN ),
		.VGA_RGB({VGA_R,VGA_G,VGA_B}),

		//HDMI
		.HDMI_CLK_P(HDMI_CLK_P),
		.HDMI_D2_P (HDMI_D2_P ),
		.HDMI_D1_P (HDMI_D1_P ),
		.HDMI_D0_P (HDMI_D0_P )	
		
	);


// ================================================================
// SD卡 BMP 文件读取模块（来自q3v3）
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
// 数码管显示模块（来自q3v3）
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
	.seg_data_0                 ({1'b1, (current_function == 2'd0) ? 7'b0000110 : (current_function == 2'd1) ? 7'b0101101 : (current_function == 2'd2) ? 7'b0100111 : 7'b0110011}), // 显示功能: 1/2/3/4
	.seg_data_1                 ({1'b1,7'b1111_111}       ),
	.seg_data_2                 ({1'b1,7'b1111_111}       ),
	.seg_data_3                 ({1'b1,7'b1111_111}       ),
	.seg_data_4                 ({1'b1,7'b1111_111}       ),
	.seg_data_5                 ({1'b1,seg_data_0}        )
);

// ================================================================
// OV5640 摄像头驱动（来自q4v4）
// ================================================================

ov5640_dri u_ov5640_dri(
	.clk                        (clk),
	.rst_n                      (rst_n),

	.cam_pclk                   (cam_pclk),
	.cam_vsync                  (cam_vsync),
	.cam_href                   (cam_href),
	.cam_data                   (cam_data),
	.cam_rst_n                  (cam_rst_n),
	.cam_pwdn                   (cam_pwdn),
	.cam_scl                    (cam_scl),
	.cam_sda                    (cam_sda),

	.capture_start              (Sdr_init_done),
	.cmos_h_pixel               (H_CMOS_DISP),
	.cmos_v_pixel               (V_CMOS_DISP),
	.total_h_pixel              (TOTAL_H_PIXEL),
	.total_v_pixel              (TOTAL_V_PIXEL),
	.cmos_frame_vsync           (cmos_frame_vsync),
	.cmos_frame_href            (cmos_frame_href),
	.cmos_frame_valid           (cmos_frame_valid),
	.cmos_frame_data            (cmos_wr_data)
);

ov5640_delay u_ov5640_delay(
	.clk                        (cam_pclk),
	.rst_n                      (rst_n),
	.cmos_frame_vsync           (cmos_frame_vsync),
	.cmos_frame_href            (cmos_frame_href),
	.cmos_frame_valid           (cmos_frame_valid),
	.cmos_wr_data               (cmos_wr_data),

	.cam_write_req              (cam_write_req),
	.cam_write_req_ack          (cam_write_req_ack),
	.cam_write_en               (cam_write_en),
	.cam_write_data             (cam_write_data),
    .output_mode                (1'b1)
);

// ================================================================
// UDP 摄像头控制模块
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
// SDRAM 帧缓存读写控制（使用多路选择的数据源）
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

	// 多路选择的写入接口（根据模式选择SD卡或摄像头）
	.write_clk                  (write_clk_sel),
	.write_req                  (write_req_sel),
	.write_req_ack              (write_req_ack_sel),
	.write_finish               (),
	.write_addr_0               (24'd0),
	.write_addr_1               (24'd0),
	.write_addr_2               (24'd0),
	.write_addr_3               (24'd0),
	.write_addr_index           (2'd0),
	.write_len                  (24'd307200),
	.write_en                   (write_en_sel),
	.write_data                 (write_data_sel)
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
// UDP/IP 协议栈
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
    .app_rx_data_valid          (app_rx_data_valid      ), 
    .app_rx_data                (app_rx_data            ), 
    .app_rx_data_length         (app_rx_data_length     ), 
    .app_rx_port_num            (app_rx_port_num        ), 

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
