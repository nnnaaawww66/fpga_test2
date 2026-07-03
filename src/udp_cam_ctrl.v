// ============================================================================
// UDP 摄像头数据传输控制模块 (RGB888 传输方案)
// 功能：从 SDRAM 读取 RGB888 图像数据，通过 UDP 协议发送到上位机
// 数据格式：每像素4字节 [R G B 0]，每包640字节，共1920包
// ============================================================================

module udp_cam_ctrl(
	input               clk,              // UDP时钟（125MHz）
	input               rst_n,

	// ===== SDRAM 读取接口（连接到 frame_read_write）=====
	output reg          read_req,         // 请求读取一帧数据
	input               read_req_ack,     // 读请求响应
	output reg          read_en,          // 读使能
	input [31:0]        read_data,        // 读取的数据（1个 RGB888 像素）

	// ===== UDP 发送接口（连接到 UDP 协议栈）=====

	input               udp_tx_ready,     // UDP 发送准备好
	input               app_tx_ack,       // UDP 应答
	output reg          app_tx_data_request,   // 请求发送数据
	output reg          app_tx_data_valid,     // 数据有效
	output reg [7:0]    app_tx_data,           // 发送的数据
	output reg [15:0]   udp_data_length        // UDP 包长度
);

// ============================================================================
// 参数定义
// ============================================================================

localparam  IMG_HEADER      = 32'hAA0055FF;     // 图像帧头标识
localparam  IMG_WIDTH       = 32'd640;          // 图像宽度
localparam  IMG_HEIGHT      = 32'd480;          // 图像高度
localparam  IMG_TOTAL       = IMG_WIDTH * IMG_HEIGHT * 4;  // RGB888 总字节数: 1,228,800字节
localparam  IMG_FRAMSIZE    = 32'd640;          // 每个 UDP 包的数据大小（字节）
localparam  IMG_FRAMTOTAL   = 32'd1920;         // 总包数 = 1,228,800/640 = 1920 包
localparam  IMG_HEADER_LEN  = 256;              // 包头长度（32字节 = 256bit）

// ============================================================================
// 状态机定义
// ============================================================================

localparam  START_UDP       = 3'd0;     // 开始新帧
localparam  WAIT_FIFO_RDY   = 3'd1;     // 等待 FIFO 准备好
localparam  WAIT_UDP_DATA   = 3'd2;     // 等待 UDP 准备好
localparam  WAIT_ACK        = 3'd3;     // 等待 UDP 应答
localparam  SEND_UDP_HEADER = 3'd4;     // 发送 UDP 包头
localparam  SEND_UDP_DATA   = 3'd5;     // 发送图像数据
localparam  DELAY           = 3'd6;     // 延迟

reg [2:0]   STATE;

// ============================================================================
// 寄存器定义
// ============================================================================

reg [31:0]  IMG_FRAMSEQ;        // 当前包序号（0 ~ 959）
reg [31:0]  IMG_PICSEQ;         // 帧序号（累加）
reg [31:0]  IMG_OFFSET;         // 当前包在整帧中的偏移量（字节）

reg [8:0]   app_tx_header_cnt;  // 包头发送计数器
reg [11:0]  data_send_cnt;      // 数据发送计数器 (0-639)
reg [21:0]  delay_cnt;          // 延迟计数器

// SDRAM 读取相关
reg [31:0]  sdram_data_reg;     // 锁存SDRAM数据
reg [1:0]   byte_select_cnt;    // 字节选择计数器 (0-3)
reg         start_read;         // 启动读取标志

// ============================================================================
// UDP 包头结构（32字节 = 256bit）
// ============================================================================

wire [255:0] UDP_HEADER_32 = {
	IMG_FRAMSIZE,   // [255:224]
	IMG_FRAMSEQ,    // [223:192]
	IMG_PICSEQ,     // [191:160]
	IMG_OFFSET,     // [159:128]
	IMG_TOTAL,      // [127:96]
	IMG_HEIGHT,     // [95:64]
	IMG_WIDTH,      // [63:32]
	IMG_HEADER      // [31:0]
};

// ============================================================================
// 主状态机
// ============================================================================

always @(posedge clk or negedge rst_n) begin
	if(!rst_n) begin
		STATE               <= START_UDP;
		app_tx_data_request <= 1'b0;
		app_tx_data_valid   <= 1'b0;
		app_tx_data         <= 8'd0;
		udp_data_length     <= 16'd672;     // 640 数据 + 32 包头
		IMG_FRAMSEQ         <= 32'd0;
		IMG_PICSEQ          <= 32'd0;
		IMG_OFFSET          <= 32'd0;
		app_tx_header_cnt   <= 9'd0;
		data_send_cnt       <= 12'd0;
		delay_cnt           <= 22'd0;
		read_req            <= 1'b0;
		read_en             <= 1'b0;
        sdram_data_reg      <= 32'd0;
        byte_select_cnt     <= 2'd0;
        start_read          <= 1'b0;
	end
	else begin
		case(STATE)
			// ========== 状态 0：开始新帧 ==========

			START_UDP: begin
				app_tx_data_request <= 1'b0;
				app_tx_data_valid   <= 1'b0;
				data_send_cnt       <= 12'd0;
				IMG_FRAMSEQ         <= 32'd0;
				IMG_OFFSET          <= 32'd0;
				read_req            <= 1'b0;
				read_en             <= 1'b0;
				IMG_PICSEQ          <= IMG_PICSEQ + 1'd1;  // 帧号递增
				delay_cnt           <= 22'd0;
				STATE               <= WAIT_FIFO_RDY;
			end

			// ========== 状态 1：等待 FIFO 准备好 ==========

			WAIT_FIFO_RDY: begin
				if(delay_cnt >= 2000) begin
					delay_cnt <= 22'd0;
					STATE     <= WAIT_UDP_DATA;
				end
				else begin
					delay_cnt <= delay_cnt + 1'd1;
					STATE     <= WAIT_FIFO_RDY;
				end

				// 延迟 10 个周期后请求读取一帧数据
				if(delay_cnt == 10)
					read_req <= 1'b1;
				else if(read_req_ack)
					read_req <= 1'b0;
			end

			// ========== 状态 2：等待 UDP 准备好 ==========

			WAIT_UDP_DATA: begin
				if(udp_tx_ready) begin
					app_tx_data_request <= 1'b1;
					STATE               <= WAIT_ACK;
				end
				else begin
					app_tx_data_request <= 1'b0;
					STATE               <= WAIT_UDP_DATA;
				end
			end

			// ========== 状态 3：等待 UDP 应答 ==========

			WAIT_ACK: begin
				if(app_tx_ack) begin
					app_tx_data_request <= 1'b0;
					app_tx_header_cnt   <= 9'd8;
					app_tx_data_valid   <= 1'b1;
					app_tx_data         <= UDP_HEADER_32[7:0];
					STATE               <= SEND_UDP_HEADER;
				end
				else begin
					app_tx_data_request <= 1'b1;
					STATE               <= WAIT_ACK;
				end
			end

			// ========== 状态 4：发送 UDP 包头（32字节）==========
			SEND_UDP_HEADER: begin
                // 在包头发送即将结束时，提前启动第一次SDRAM读取
                if(app_tx_header_cnt == 248) begin // 倒数第2个字节
                    start_read <= 1'b1;
                end else begin
                    start_read <= 1'b0;
                end

				if(app_tx_header_cnt >= IMG_HEADER_LEN) begin
					// 包头发送完成
					STATE              <= SEND_UDP_DATA;
					app_tx_data_valid  <= 1'b1; // 保持valid
					app_tx_data        <= UDP_HEADER_32[app_tx_header_cnt +: 8];
					app_tx_header_cnt  <= 9'd0;
					data_send_cnt      <= 12'd0;
                    byte_select_cnt    <= 2'd0;
				end
				else begin
					STATE             <= SEND_UDP_HEADER;
					app_tx_data_valid <= 1'b1;
					app_tx_data       <= UDP_HEADER_32[app_tx_header_cnt +: 8];
					app_tx_header_cnt <= app_tx_header_cnt + 8;
				end
			end

			// ========== 状态 5：发送图像数据（640字节）==========
			SEND_UDP_DATA: begin
                // SDRAM 读控制 (2周期延迟)
                // start_read -> read_en -> (1 cycle) -> (1 cycle) -> data valid
                read_en <= start_read; // read_en会比start_read延迟一个周期

                // 数据锁存
                // 当read_en为高时，表示2个周期后数据会有效
                // 我们在read_en变高后第3个周期锁存数据
                if (read_en) begin
                    sdram_data_reg <= read_data;
                end

                // 字节选择与发送
                case(byte_select_cnt)
                    2'd0: app_tx_data <= sdram_data_reg[7:0];
                    2'd1: app_tx_data <= sdram_data_reg[15:8];
                    2'd2: app_tx_data <= sdram_data_reg[23:16];
                    2'd3: app_tx_data <= sdram_data_reg[31:24];
                endcase

                // 计数器和状态更新
				if(data_send_cnt >= (IMG_FRAMSIZE - 1)) begin
					// 一包发送完成 (640字节)
					data_send_cnt      <= 12'd0;
					app_tx_data_valid  <= 1'b0;
                    start_read         <= 1'b0;
                    read_en            <= 1'b0;
					STATE              <= DELAY;
				end
				else begin
					// 继续发送当前包
                    data_send_cnt <= data_send_cnt + 1'd1;

                    // 在发送完一个32位字的最后一个字节后，启动下一次读取
                    if (byte_select_cnt == 2'd3) begin
                        byte_select_cnt <= 2'd0;
                        start_read <= 1'b1;
                    end else begin
                        byte_select_cnt <= byte_select_cnt + 1'd1;
                        start_read <= 1'b0;
                    end

					app_tx_data_valid <= 1'b1;
					STATE             <= SEND_UDP_DATA;
				end
			end

			// ========== 状态 6：包间延迟 ==========

			DELAY: begin
				if(delay_cnt >= 800) begin
					delay_cnt  <= 22'd0;
					IMG_FRAMSEQ <= IMG_FRAMSEQ + 1'd1;
					IMG_OFFSET  <= IMG_OFFSET + IMG_FRAMSIZE;

					// 判断是否发送完整帧
					if(IMG_FRAMSEQ >= (IMG_FRAMTOTAL - 1))
						STATE <= START_UDP;  // 开始下一帧
					else
						STATE <= WAIT_UDP_DATA;  // 继续发送下一包
				end
				else begin
					delay_cnt <= delay_cnt + 1'd1;
					STATE     <= DELAY;
				end
			end

			default: STATE <= START_UDP;
		endcase
	end
end

endmodule