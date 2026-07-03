module ov5640_delay( 
	input 					clk,
    input					rst_n,
	
	input 					cmos_frame_vsync,
	input 					cmos_frame_href,
	input 					cmos_frame_valid,
	input 		[15:0] 		cmos_wr_data,
    
	output             		cam_write_en,
	output 		[31:0]      cam_write_data,
	output  reg             cam_write_req,
	input               	cam_write_req_ack,
	input               	output_mode // 0:原始RGB, 1:肤色检测框选
);
reg cmos_frame_href_d0;
reg cmos_frame_vsync_d0;
reg cmos_frame_valid_d0;
reg [15:0] cmos_wr_data_d0;
reg cmos_frame_href_d1;
reg cmos_frame_vsync_d1;
reg cmos_frame_valid_d1;
reg [15:0] cmos_wr_data_d1;

wire [7:0] cam_R,cam_G,cam_B;
assign cam_R = cmos_wr_data_d1[15:11] << 3; // 左移缩放扩展至8位
assign cam_G = cmos_wr_data_d1[10:5] << 2;  // 左移缩放扩展至8位
assign cam_B = cmos_wr_data_d1[4:0] << 3;   // 左移缩放扩展至8位

// === 通道重排 ===
wire [7:0] correct_R, correct_G, correct_B;
// 移除摄像头通道交换，使用原始RGB通道
assign correct_R = cam_R;  // 直接使用原始R通道
assign correct_G = cam_G;  // 直接使用原始G通道
assign correct_B = cam_B;  // 直接使用原始B通道

// YCbCr转换与肤色检测流水线
reg [15:0] r_d0, g_d0, b_d0;
reg [15:0] r_d1, g_d1, b_d1;
reg [15:0] r_d2, g_d2, b_d2;
reg [15:0] y_d0, cb_d0, cr_d0;
reg [7:0] y_d1, cb_d1, cr_d1;
reg is_skin_region;

// 像素位置计数器
reg [10:0] pixel_x, pixel_y;
reg href_prev, vsync_prev;

always@(posedge clk or negedge rst_n) begin
    if(!rst_n) begin
        pixel_x <= 0;
        pixel_y <= 0;
        href_prev <= 0;
        vsync_prev <= 0;
    end else begin
        href_prev <= cmos_frame_href_d1;
        vsync_prev <= cmos_frame_vsync_d1;
        
        // 场同步上升沿，重置行计数器
        if(cmos_frame_vsync_d1 && !vsync_prev) begin
            pixel_y <= 0;
        end
        // 行同步上升沿，重置列计数器，增加行计数器
        else if(cmos_frame_href_d1 && !href_prev) begin
            pixel_x <= 0;
            pixel_y <= pixel_y + 1;
        end
        // 有效像素，增加列计数器
        else if(cmos_frame_valid_d1) begin
            pixel_x <= pixel_x + 1;
        end
    end
end

// YCbCr转换流水线
always@(posedge clk or negedge rst_n) begin
    if(!rst_n) begin
        r_d0 <= 0; g_d0 <= 0; b_d0 <= 0;
        r_d1 <= 0; g_d1 <= 0; b_d1 <= 0;
        r_d2 <= 0; g_d2 <= 0; b_d2 <= 0;
    end else begin
        // 使用正确的颜色顺序进行YCbCr转换
        r_d0 <= 66 * correct_R;
        g_d0 <= 129 * correct_G;
        b_d0 <= 25 * correct_B;
        r_d1 <= 38 * correct_R;
        g_d1 <= 74 * correct_G;
        b_d1 <= 112 * correct_B;
        r_d2 <= 112 * correct_R;
        g_d2 <= 94 * correct_G;
        b_d2 <= 18 * correct_B;
    end
end

always@(posedge clk or negedge rst_n) begin
    if(!rst_n) begin
        y_d0 <= 0; cb_d0 <= 0; cr_d0 <= 0;
    end else begin
        y_d0 <= r_d0 + g_d0 + b_d0 + 4096;
        cb_d0 <= b_d1 - r_d1 - g_d1 + 32768;
        cr_d0 <= r_d2 - g_d2 - b_d2 + 32768;
    end
end

always@(posedge clk or negedge rst_n) begin
    if(!rst_n) begin
        y_d1 <= 0; cb_d1 <= 0; cr_d1 <= 0;
    end else begin
        y_d1 <= y_d0[15:8];
        cb_d1 <= cb_d0[15:8];
        cr_d1 <= cr_d0[15:8];
    end
end

// 肤色检测 - 调试版本1（放宽范围）
always@(posedge clk or negedge rst_n) begin
    if(!rst_n) begin
        is_skin_region <= 0;
    end else begin
        // 先放宽范围确认能检测到
        if(cb_d1 > 90 && cb_d1 < 125 && 
           cr_d1 > 135 && cr_d1 < 165) begin
            is_skin_region <= 1;
        end else begin
            is_skin_region <= 0;
        end
    end
end

// 肤色区域框选逻辑
reg [7:0] boxed_R, boxed_G, boxed_B;

always@(posedge clk or negedge rst_n) begin
    if(!rst_n) begin
        boxed_R <= 0;
        boxed_G <= 0;
        boxed_B <= 0;
    end else begin
        if(is_skin_region) begin
            // 肤色区域用红色边框标记
            // 检查是否是边界像素（创建边框效果）
            if(pixel_x < 5 || pixel_x > 635 || pixel_y < 5 || pixel_y > 475 ||
               (pixel_x % 20 < 2) || (pixel_y % 20 < 2)) begin
                // 红色边框
                boxed_R <= 8'hFF;
                boxed_G <= 8'h00;
                boxed_B <= 8'h00;
            end else begin
                // 内部保持原色
                boxed_R <= correct_R;
                boxed_G <= correct_G;
                boxed_B <= correct_B;
            end
        end else begin
            // 非肤色区域保持原色
            boxed_R <= correct_R;
            boxed_G <= correct_G;
            boxed_B <= correct_B;
        end
    end
end

// 输出选择逻辑 - 添加8位填充确保32位对齐
assign cam_write_data = output_mode ? 
    {boxed_B, boxed_G, boxed_R, 8'd0} :  // BGR+Padding (32位对齐)
    {cam_B, cam_G, cam_R, 8'd0};         // BGR+Padding (32位对齐)

assign cam_write_en = cmos_frame_valid_d1;

// 原有逻辑保持不变
always@(posedge clk or negedge rst_n) begin
	if(~rst_n) begin
		cmos_frame_href_d0 <= 1'b0;
		cmos_frame_vsync_d0 <= 1'b0;
		cmos_frame_valid_d0 <= 1'b0;
        cmos_wr_data_d0 <= 16'd0;
        cmos_wr_data_d1 <= 16'd0;
	end else begin
		cmos_frame_href_d0 <= cmos_frame_href;
		cmos_frame_vsync_d0 <= cmos_frame_vsync;
		cmos_frame_valid_d0 <= cmos_frame_valid;
        cmos_wr_data_d0 <= cmos_wr_data;
        
		cmos_frame_href_d1 <= cmos_frame_href_d0;
		cmos_frame_vsync_d1 <= cmos_frame_vsync_d0;
		cmos_frame_valid_d1 <= cmos_frame_valid_d0;	
		cmos_wr_data_d1 <= cmos_wr_data_d0;
    end
end

always@(posedge clk or negedge rst_n) begin
	if(~rst_n) begin
		cam_write_req <= 1'b0;
    end
	else if(cmos_frame_vsync_d0 & ~cmos_frame_vsync) begin
		cam_write_req <= 1'b1;
    end
    else if(cam_write_req_ack) begin
		cam_write_req <= 1'b0;
    end
end

endmodule