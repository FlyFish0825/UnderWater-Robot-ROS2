"""Loopback fixtures validate the acceptance client; no hardware evidence."""
# 作用：test_tcp_forwarding.py（验收客户端）的自测——用回环 socket 与固定报文样本验证
#   帧解析、CRC 容错、UWSC 头部校验、JPEG 解码，以及 probe() 的并发/重连与“绝不发送指令”行为。
# 运行方式：在 tools/ 目录下执行 python -m unittest test_tcp_forwarding_client -v，
#   也可直接 python tools/test_tcp_forwarding_client.py -v（依赖同目录模块与 Pillow）。
# 通过标准：unittest 全部用例通过（退出码 0）；任一断言失败即输出 FAIL/ERROR 并返回非 0。
# 是否需要真实硬件：不需要。全程只在 127.0.0.1 回环与 socketpair 上跑，不接触机器人。
import io
import socket
import struct
import threading
import time
import unittest

from PIL import Image
import test_tcp_forwarding as client

# 三条真实抓取的样本帧，分别覆盖 0x58 心跳、0x55 CAN、0x5B 信息三种家族。
HEARTBEAT = bytes.fromhex('aa5801010000150000000500b873f2b30000000000da2158aa')
CAN = bytes.fromhex('aa55183b0101020000020c00000000f300bb4500033600c80631003f55aa')
INFO = bytes.fromhex('aa5b010101017856341200000000000052d65baa')


def jpeg_bytes():
    """生成一张 64x32 的最小合法 JPEG，作为相机帧载荷。"""
    out = io.BytesIO()
    Image.new('RGB', (64, 32), (20, 80, 140)).save(out, format='JPEG')
    return out.getvalue()


def packet(jpeg, seq=1, width=64, **fields):
    """按 UWSC v1 组装一帧：40 字节网络序头 + JPEG；fields 用于覆盖 magic/version/size 以测异常分支。"""
    return struct.pack('!4sHHQQIIII', fields.get('magic', b'UWSC'), fields.get('version', 1),
                       1, seq, seq * 1000000, width, 32, fields.get('size', len(jpeg)), 0) + jpeg


class FixtureServer:
    """回环假服务器：接受连接后持续推送 payload(sequence) 生成的字节，并记录收到的数据。"""

    def __init__(self, payload):
        self.payload = payload
        self.stop = threading.Event()
        self.listener = socket.socket()
        self.listener.bind(('127.0.0.1', 0))  # 端口 0 = 由系统分配空闲端口
        self.listener.listen(4)
        self.listener.settimeout(0.1)  # 短超时便于响应 stop 事件
        self.port = self.listener.getsockname()[1]
        self.connections = 0  # 用于断言 probe() 确实各重连一次
        self.received = bytearray()
        self.thread = threading.Thread(target=self.run)
        self.thread.start()

    def run(self):
        sequence = 0
        while not self.stop.is_set():
            try:
                conn, _ = self.listener.accept()
            except socket.timeout:
                continue
            with conn:
                self.connections += 1
                conn.settimeout(0.001)
                while not self.stop.is_set():
                    try:
                        sequence += 1
                        data = self.payload(sequence)
                        # Force multiple small writes across header/body boundaries.
                        for offset in range(0, len(data), 17):  # 每次只发 17 字节，强制客户端处理分片
                            conn.sendall(data[offset:offset + 17])
                        try:
                            data = conn.recv(4096)  # 收下客户端发来的任何字节，供“未发送指令”断言
                            if not data:
                                break
                            self.received.extend(data)
                        except socket.timeout:
                            pass
                        self.stop.wait(0.025)  # 约 40 帧/秒
                    except OSError:
                        break
        self.listener.close()

    def close(self):
        """停止服务线程；2 秒内未退出即报错，避免测试挂死。"""
        self.stop.set()
        self.thread.join(timeout=2)
        if self.thread.is_alive():
            raise RuntimeError('Fixture did not stop')


class AcceptanceClientTests(unittest.TestCase):
    def test_known_protocol_samples_all_fragment_boundaries(self):
        """被测行为：RawFrames 在任意切分点喂入数据都应还原出同一帧。期望：各家族各计 1 帧且无 CRC 错误。"""
        for sample, family in ((HEARTBEAT, '58'), (CAN, '55'), (INFO, '5B')):
            for cut in range(len(sample) + 1):  # 遍历全部 0..len 切分点，覆盖帧头/帧尾被拆开的情况
                parser = client.RawFrames()
                parser.feed(sample[:cut])
                parser.feed(sample[cut:])
                self.assertEqual(parser.frames, {family: 1})
                self.assertEqual(parser.crc_errors, 0)

    def test_crc_corruption_detected_and_next_frame_recovered(self):
        """被测行为：翻转心跳帧第 16 字节制造 CRC 错误。期望：记 1 次 CRC 错误，且能继续解析出紧随其后的正常帧。"""
        corrupt = bytearray(HEARTBEAT)
        corrupt[16] ^= 1
        parser = client.RawFrames()
        parser.feed(corrupt + HEARTBEAT)
        self.assertEqual(parser.crc_errors, 1)
        self.assertEqual(parser.frames, {'58': 1})

    def decode(self, data):
        """把整段字节写进 socketpair 后调用 camera_frame 解码，模拟单帧到达。"""
        a, b = socket.socketpair()
        with a, b:
            a.sendall(data)
            a.shutdown(socket.SHUT_WR)  # 关闭写端，让读取方遇到完整流结束
            return client.camera_frame(b)

    def test_camera_header_and_decoded_dimensions(self):
        """被测行为：解码一帧合法 UWSC 数据。期望：返回 (序号 1, 时间戳 1e6, 宽 64, 高 32)。"""
        self.assertEqual(self.decode(packet(jpeg_bytes()))[:4], (1, 1000000, 64, 32))

    def test_camera_invalid_headers_rejected_before_payload_read(self):
        """被测行为：magic 错误、版本错误、载荷超 8MiB、载荷为 0 四种非法头部。期望：均抛 ValueError。"""
        for fields in ({'magic': b'HTTP'}, {'version': 2}, {'size': 8 * 1024 * 1024 + 1}, {'size': 0}):
            with self.subTest(fields=fields), self.assertRaises(ValueError):
                self.decode(packet(b'', **fields))

    def test_camera_jpeg_corruption_and_mismatched_dimensions(self):
        """被测行为：非 JPEG 载荷、头部宽度与实际不符、帧被截断。期望：分别抛异常 / ValueError / EOFError。"""
        with self.assertRaises(Exception):
            self.decode(packet(b'not a JPEG'))
        with self.assertRaises(ValueError):
            self.decode(packet(jpeg_bytes(), width=66))
        with self.assertRaises(EOFError):
            self.decode(packet(jpeg_bytes())[:-10])

    def test_concurrent_streams_and_reconnect_send_no_commands(self):
        """被测行为：对两个假服务器跑完整 probe()。期望：结果 PASS、四条会话记录、各重连 1 次且服务端未收到任何字节。"""
        jpeg = jpeg_bytes()
        raw = FixtureServer(lambda _: HEARTBEAT + CAN + INFO)  # 原始流每轮推三种家族样本
        camera = FixtureServer(lambda seq: packet(jpeg, seq))
        try:
            report = client.probe('127.0.0.1', raw.port, camera.port, 2)
            self.assertEqual(report['result'], 'PASS', report)
            self.assertEqual(len(report['sessions']), 4)  # concurrent/reconnect × raw/camera
            self.assertEqual(raw.connections, 2)
            self.assertEqual(camera.connections, 2)
            self.assertFalse(raw.received or camera.received)  # 验收客户端只收不发
        finally:
            raw.close()
            camera.close()


if __name__ == '__main__':
    unittest.main(verbosity=2)
