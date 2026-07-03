`timescale 1ns/1ps
module udp_receive_img #(
    parameter DEVICE = "EG4",
    parameter ADDR_WIDTH = 24,
    parameter DATA_WIDTH = 32
)(
    input                       clk,
    input                       rst_n,
    
    // UDP 接收接口
    input                       app_rx_data_valid,
    input [7:0]                 app_rx_data,
    input [15:0]                app_rx_data_length,
    input [15:0]                app_rx_port_num,
    
    // SDRAM 写入接口
    output reg                  write_req,
    input                       write_req_ack,
    output reg                  write_en,
    output reg [DATA_WIDTH-1:0] write_data,
    
    // LED 控制接口
    output reg [7:0]            led_ctrl
);

// 状态机定义
localparam IDLE            = 3'd0;
localparam WAIT_HEADER     = 3'd1;
localparam PARSE_HEADER    = 3'd2;
localparam RECEIVE_DATA    = 3'd3;
localparam SEND_REQ        = 3'd4;
localparam WRITE_SDRAM     = 3'd5;
localparam FINISH          = 3'd6;

// 命令类型定义
localparam CMD_LED         = 8'h01;  // LED控制命令
localparam CMD_IMAGE       = 8'h02;  // 图像数据命令

// 内部寄存器
reg [2:0]                   state;
reg [15:0]                  data_cnt;
reg [7:0]                   cmd_type;
reg [15:0]                  img_width;
reg [15:0]                  img_height;
reg [2:0]                   byte_cnt;
reg [23:0]                  buffer_data;
reg [15:0]                  pixel_cnt;

// 图像处理参数
localparam MAX_PIXELS      = 640 * 480;  // 最大像素数 (640x480)

// 状态机控制逻辑
always @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
        state           <= IDLE;
        data_cnt        <= 16'd0;
        cmd_type        <= 8'd0;
        img_width       <= 16'd640;
        img_height      <= 16'd480;
        byte_cnt        <= 3'd0;
        buffer_data     <= 24'd0;
        pixel_cnt       <= 16'd0;
        write_req       <= 1'b0;
        write_en        <= 1'b0;
        write_data      <= {DATA_WIDTH{1'b0}};
        led_ctrl        <= 8'h00;
    end else begin
        case (state)
            IDLE: begin
                if (app_rx_data_valid) begin
                    state <= WAIT_HEADER;
                    data_cnt <= 16'd1;
                end
            end
            
            WAIT_HEADER: begin
                if (app_rx_data_valid) begin
                    data_cnt <= data_cnt + 16'd1;
                    if (data_cnt == 16'd1) begin
                        cmd_type <= app_rx_data;
                    end
                    
                    // 处理LED控制命令
                    if (cmd_type == CMD_LED && data_cnt >= 16'd2 && data_cnt <= 16'd9) begin
                        led_ctrl[data_cnt - 16'd2] <= app_rx_data[0];
                        if (data_cnt == 16'd9) begin
                            state <= IDLE;
                        end
                    end
                    
                    // 处理图像数据命令头部
                    else if (cmd_type == CMD_IMAGE) begin
                        if (data_cnt == 16'd2) begin
                            img_width[7:0] <= app_rx_data;
                        end else if (data_cnt == 16'd3) begin
                            img_width[15:8] <= app_rx_data;
                        end else if (data_cnt == 16'd4) begin
                            img_height[7:0] <= app_rx_data;
                        end else if (data_cnt == 16'd5) begin
                            img_height[15:8] <= app_rx_data;
                            state <= RECEIVE_DATA;
                            byte_cnt <= 3'd0;
                            pixel_cnt <= 16'd0;
                        end
                    end
                    
                    // 数据包结束
                    if (data_cnt == app_rx_data_length) begin
                        state <= IDLE;
                    end
                end
            end
            
            RECEIVE_DATA: begin
                if (app_rx_data_valid) begin
                    // 收集RGB数据（每3字节一个像素）
                    buffer_data[(23 - byte_cnt * 8) -: 8] <= app_rx_data;
                    byte_cnt <= byte_cnt + 3'd1;
                    
                    // 每收集到一个完整的RGB像素（3字节），准备写入SDRAM
                    if (byte_cnt == 3'd2) begin
                        state <= SEND_REQ;
                        write_req <= 1'b1;
                    end
                    
                    // 数据包结束
                    if (data_cnt == app_rx_data_length) begin
                        state <= IDLE;
                    end
                end
            end
            
            SEND_REQ: begin
                if (write_req_ack) begin
                    write_req <= 1'b0;
                    state <= WRITE_SDRAM;
                    write_en <= 1'b1;
                    // 将RGB565数据转换为32位格式（假设高16位为0，低16位为RGB565）
                    write_data <= {{16{1'b0}}, 
                                  {buffer_data[23:19], buffer_data[15:10], buffer_data[7:3]}};
                    pixel_cnt <= pixel_cnt + 16'd1;
                end
            end
            
            WRITE_SDRAM: begin
                write_en <= 1'b0;
                byte_cnt <= 3'd0;
                
                // 检查是否继续接收数据
                if (app_rx_data_valid) begin
                    state <= RECEIVE_DATA;
                    data_cnt <= data_cnt + 16'd1;
                    buffer_data[(23 - byte_cnt * 8) -: 8] <= app_rx_data;
                    byte_cnt <= byte_cnt + 3'd1;
                end else if (pixel_cnt >= MAX_PIXELS || data_cnt >= app_rx_data_length) begin
                    state <= FINISH;
                end
            end
            
            FINISH: begin
                state <= IDLE;
                pixel_cnt <= 16'd0;
            end
        endcase
    end
end

endmodule