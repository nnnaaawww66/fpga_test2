import socket
import struct
import time
import threading
import numpy as np
import cv2
import queue
from tkinter import Tk, Button, Label, Frame, Entry, StringVar, Text, Scrollbar, END, LEFT, RIGHT, BOTH, X, Y


DEFAULT_HOST_IP = "192.168.240.2"
DEFAULT_HOST_PORT = 6003
BUFFER_SIZE = 8192

IMG_WIDTH = 640
IMG_HEIGHT = 480
PACKET_HEADER_SIZE = 32
PACKET_DATA_SIZE = 636
TOTAL_PACKETS_PER_FRAME = 1450  

FRAME_TIMEOUT = 2.0


class UDPCameraReceiver:
  

    def __init__(self, host_ip, host_port, packet_queue):
        self.host_ip = host_ip
        self.host_port = host_port
        self.packet_queue = packet_queue
        self.sock = None
        self.running_event = threading.Event()

    def start(self):
        try:
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 16 * 1024 * 1024)
            self.sock.bind((self.host_ip, self.host_port))
            self.sock.settimeout(1.0)
            self.running_event.set()
            threading.Thread(target=self._receive_loop, daemon=True).start()
            print(f"[INFO] 接收线程已启动，监听 {self.host_ip}:{self.host_port}")
            return True
        except Exception as e:
            print(f"[ERROR] 无法启动UDP接收器: {e}")
            return False

    def stop(self):
        self.running_event.clear()
        if self.sock:
            self.sock.close()
        print("[INFO] 接收线程已停止")

    def _receive_loop(self):
        while self.running_event.is_set():
            try:
                data, _ = self.sock.recvfrom(BUFFER_SIZE)
                if data:
                    try:
                        self.packet_queue.put_nowait(data)
                    except queue.Full:
                        pass
            except socket.timeout:
                continue
            except Exception:
                if self.running_event.is_set():
                    print(f"[ERROR] 网络接收异常")
                break


class CameraViewerApp:
   

    def __init__(self, root):
        self.root = root
        self.root.title("FPGA 摄像头 (直接RGB输出)")
        self.root.geometry("800x400")

        self.receiver = None
        self.is_running = False
        self.packet_queue = queue.Queue(maxsize=TOTAL_PACKETS_PER_FRAME * 2)

        self.frame_packets = {}
        self.current_frame_seq = -1
        self.last_packet_time = 0

        self.fps = 0.0
        self.last_fps_time = time.time()
        self.fps_frame_count = 0

        self._setup_ui()
        self.root.protocol("WM_DELETE_WINDOW", self.on_closing)
        self.log("程序就绪，请点击开始接收。")

    def _setup_ui(self):
        top_frame = Frame(self.root, padx=10, pady=10)
        top_frame.pack(fill=X)
        Label(top_frame, text="本机IP:").pack(side=LEFT)
        self.host_ip_var = StringVar(value=DEFAULT_HOST_IP)
        Entry(top_frame, textvariable=self.host_ip_var, width=15).pack(side=LEFT)
        Label(top_frame, text="端口:").pack(side=LEFT, padx=(10, 5))
        self.host_port_var = StringVar(value=str(DEFAULT_HOST_PORT))
        Entry(top_frame, textvariable=self.host_port_var, width=6).pack(side=LEFT)
        self.start_button = Button(top_frame, text="开始接收", command=self.start_receiving, width=12, bg="#4CAF50",
                                   fg="white")
        self.start_button.pack(side=LEFT, padx=(20, 5))
        self.stop_button = Button(top_frame, text="停止接收", command=self.stop_receiving, width=12, bg="#F44336",
                                  fg="white", state="disabled")
        self.stop_button.pack(side=LEFT, padx=5)

        self.status_label = Label(self.root, text="状态: 等待开始", bd=1, relief="sunken", anchor="w")
        self.status_label.pack(fill=X, padx=10, pady=5)

        log_frame = Frame(self.root, padx=10, pady=5)
        log_frame.pack(fill=BOTH, expand=True)
        Label(log_frame, text="日志:", anchor="w").pack(fill=X)
        self.log_text = Text(log_frame, height=8, font=("Courier New", 9))
        self.log_text.pack(fill=BOTH, expand=True)

    def start_receiving(self):
        host_ip = self.host_ip_var.get()
        host_port = int(self.host_port_var.get())

        self.frame_packets.clear()
        while not self.packet_queue.empty():
            self.packet_queue.get()

        self.receiver = UDPCameraReceiver(host_ip, host_port, self.packet_queue)
        if not self.receiver.start():
            self.log("错误: 无法启动UDP接收器")
            return

        self.is_running = True
        self.start_button.config(state="disabled")
        self.stop_button.config(state="normal")
        self.update_status(f"正在接收...")
        self.log(f"开始接收...")

        cv2.namedWindow("FPGA Camera Direct RGB", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("FPGA Camera Direct RGB", IMG_WIDTH, IMG_HEIGHT)

        self.root.after(20, self.process_and_display)

    def stop_receiving(self):
        if not self.is_running: return
        self.is_running = False
        if self.receiver:
            self.receiver.stop()

        cv2.destroyAllWindows()
        self.start_button.config(state="normal")
        self.stop_button.config(state="disabled")
        self.update_status("已停止")
        self.log("接收已停止")

    def process_and_display(self):
        if not self.is_running:
            return

        try:
            while not self.packet_queue.empty():
                packet = self.packet_queue.get_nowait()
                self._handle_packet(packet)
        except queue.Empty:
            pass

        if self.current_frame_seq != -1 and time.time() - self.last_packet_time > FRAME_TIMEOUT:
            self.log(f"超时: 帧 {self.current_frame_seq} 未收完，已丢弃。")
            self.frame_packets.clear()
            self.current_frame_seq = -1

        if cv2.waitKey(1) & 0xFF == ord('q'):
            self.stop_receiving()
            return

        self.root.after(20, self.process_and_display)

    def _handle_packet(self, packet):
        try:
            if len(packet) < PACKET_HEADER_SIZE: return

            header = struct.unpack("<8I", packet[:PACKET_HEADER_SIZE])

            if header[0] != 0xAA0055FF: return

            frame_seq, packet_seq = header[5], header[6]

        except (struct.error, IndexError):
            return

        if frame_seq != self.current_frame_seq:
            self.current_frame_seq = frame_seq
            self.frame_packets.clear()
            self.log(f"开始接收新的一帧: {frame_seq}")

        self.frame_packets[packet_seq] = packet[PACKET_HEADER_SIZE:]
        self.last_packet_time = time.time()

        if len(self.frame_packets) == TOTAL_PACKETS_PER_FRAME:
            self.log(f"帧 {self.current_frame_seq} 接收完毕, 共 {len(self.frame_packets)} 包。开始合成...")
            self._assemble_and_display_frame()
            self.frame_packets.clear()
            self.current_frame_seq = -1

    def _assemble_and_display_frame(self):
        full_frame_data = bytearray()
        for i in range(TOTAL_PACKETS_PER_FRAME):
            full_frame_data.extend(self.frame_packets.get(i, b'\x00' * PACKET_DATA_SIZE))

        expected_size = IMG_HEIGHT * IMG_WIDTH * 3
        if len(full_frame_data) < expected_size:
            self.log(f"错误：数据不完整，期望 {expected_size} 字节，实际 {len(full_frame_data)}")
            return

        final_data = full_frame_data[:expected_size]

        try:
            # 按3字节/像素解析数据
            rgb_frame = np.frombuffer(final_data, dtype=np.uint8).reshape((IMG_HEIGHT, IMG_WIDTH, 3))

            # 尝试方案：[0,2,1] = RBG顺序
            # 可尝试的组合：[0,1,2]原始 [2,1,0]BGR [1,0,2]GRB [0,2,1]RBG [1,2,0]GBR [2,0,1]BRG
            bgr_frame = rgb_frame[:, :, [0, 2, 1]].copy()


            self.fps_frame_count += 1
            elapsed = time.time() - self.last_fps_time
            if elapsed >= 1.0:
                self.fps = self.fps_frame_count / elapsed
                self.last_fps_time = time.time()
                self.fps_frame_count = 0
                self.update_status(f"正在接收... FPS: {self.fps:.1f}")

            fps_text = f"FPS: {self.fps:.1f}"
           
            cv2.putText(bgr_frame, fps_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

           
            cv2.imshow("FPGA Camera Direct RGB", bgr_frame)

        except Exception as e:
            self.log(f"图像处理错误: {e}")

    def log(self, message):
        timestamp = time.strftime("%H:%M:%S")
        self.log_text.insert(END, f"[{timestamp}] {message}\n")
        self.log_text.see(END)

    def update_status(self, text):
        self.status_label.config(text=f"状态: {text}")

    def on_closing(self):
        self.stop_receiving()
        self.root.destroy()


def main():
    root = Tk()
    app = CameraViewerApp(root)
    root.mainloop()


if __name__ == "__main__":
    main()
