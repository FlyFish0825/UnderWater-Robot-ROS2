"""Read-only LAN acceptance test; test tooling, not robot-side transport.

Runs the two ports concurrently and reconnects both. Requires Pillow on client.
Never sends bytes to the robot. A PASS proves receipt/format, not motor motion.
"""
# 作用：机器人侧 TCP 转发的局域网验收测试——并发接收原始串口流（默认 9000）与相机流（默认 9001），
#   并对两条流各做一次重连；只做接收与解码校验，不发送任何字节。
# 运行方式：python tools/test_tcp_forwarding.py --host 192.168.20.70 [--duration 10]，
#   客户端需安装 Pillow 以校验 JPEG；机器人侧必须已在运行对应 TCP 服务。
# 通过标准：全部 4 个会话（concurrent/reconnect × raw/camera）均为 PASS；报告写入
#   artifacts/tcp-repair/lan-acceptance.json，进程退出码 0 = PASS、1 = FAIL、2 = 参数错误。
# 是否需要真实硬件：需要真实机器人及其两个 TCP 端口；本脚本绝不发送控制指令，PASS 只证明“收到且格式正确”。
import argparse
import concurrent.futures
import io
import json
from pathlib import Path
import socket
import struct
import time


def crc8(data):
    # CRC-8，多项式 0x07；用于 0x55（CAN）家族的帧尾校验。逐字节异或后再跑 8 位。
    crc = 0
    for value in data:
        crc ^= value
        for _ in range(8):
            crc = ((crc << 1) ^ (7 if crc & 128 else 0)) & 255
    return crc


def crc16(data):
    # CRC-16/CCITT-FALSE，初值 0xFFFF、多项式 0x1021；用于 0x58/0x59/0x5B 家族，按小端写回帧内。
    crc = 65535
    for value in data:
        crc ^= value << 8
        for _ in range(8):
            crc = ((crc << 1) ^ (0x1021 if crc & 32768 else 0)) & 65535
    return crc


class RawFrames:
    """增量式原始帧解析器：喂入任意分片的字节流，按家族统计校验通过的帧数。"""

    def __init__(self):
        self.buffer = bytearray()
        self.frames = {}  # 键为家族 hex（'55'/'58'/'59'/'5B'），值为通过校验的帧数
        self.crc_errors = 0

    def feed(self, data):
        # 逐字节滑动同步：帧头固定为 AA + 家族字节，不符就丢弃首字节重找起点。
        self.buffer.extend(data)
        while len(self.buffer) >= 3:
            b = self.buffer
            if b[0] != 0xAA or b[1] not in (0x55, 0x58, 0x59, 0x5B):
                del b[0]
                continue
            family = b[1]
            if family == 0x55:
                # 0x55（CAN）帧：第 3 字节是 DLC，总长 = DLC + 6。
                if not 8 <= b[2] <= 76:
                    del b[0]
                    continue
                size = b[2] + 6
            else:
                # 其余家族：偏移 10..12 是小端载荷长度，总长 = 长度 + 20；4116 为上限。
                if len(b) < 16:
                    break
                size = int.from_bytes(b[10:12], 'little') + 20
                if size > 4116:
                    del b[0]
                    continue
            if len(b) < size:
                break  # 帧未收全，等下一次 feed
            packet = bytes(b[:size])
            if packet[-2:] != bytes((family, 0xAA)):
                del b[0]
                continue
            # 0x55 家族用 1 字节 CRC8（倒数第 3 字节），其余用 2 字节小端 CRC16（倒数 4..2 字节）。
            valid = (crc8(packet[2:-3]) == packet[-3] if family == 0x55 else
                     crc16(packet[1:-4]) == int.from_bytes(packet[-4:-2], 'little'))
            if not valid:
                self.crc_errors += 1
                del b[0]
                continue
            # 0x58（心跳）另有固定头 01 01 00 00，且长度只能是 20 或 25。
            if family == 0x58 and (packet[2:6] != b'\x01\x01\x00\x00' or size not in (20, 25)):
                del b[0]
                continue
            self.frames[f'{family:02X}'] = self.frames.get(f'{family:02X}', 0) + 1
            del b[:size]


def exact(sock, size):
    """从 socket 精确读取 size 字节；流提前关闭则抛 EOFError。"""
    data = bytearray()
    while len(data) < size:
        part = sock.recv(min(8191, size - len(data)))
        if not part:
            raise EOFError('Stream closed before a complete frame')
        data.extend(part)
    return bytes(data)


def camera_frame(sock):
    """读取并校验一个 UWSC v1 相机帧：40 字节头 + JPEG 载荷，返回序号/时间戳/宽高/字节数。"""
    from PIL import Image  # 延迟导入：仅相机校验需要 Pillow
    h = struct.unpack('!4sHHQQIIII', exact(sock, 40))  # 网络序头：magic,version,flags,seq,stamp,w,h,size,reserved
    magic, version, flags, sequence, stamp, width, height, size, reserved = h
    # 头部约束：magic 必须是 UWSC、版本 1、flags 1、保留字段 0；宽度 2..8192 且为偶数（双目拼接图）；
    # 高度 1..4096；JPEG 载荷 1B..8MiB。任一不满足即拒绝该会话。
    if (magic != b'UWSC' or version != 1 or flags != 1 or reserved != 0 or
            not 2 <= width <= 8192 or width % 2 or not 1 <= height <= 4096 or
            not 0 < size <= 8 * 1024 * 1024):
        raise ValueError('Invalid UWSC v1 header')
    jpeg = exact(sock, size)
    with Image.open(io.BytesIO(jpeg)) as image:
        if image.format != 'JPEG' or image.size != (width, height):
            raise ValueError('JPEG format/size does not match header')
        image.load()  # 完整解码，确保不是只截了头部
    return sequence, stamp, width, height, size


def raw_session(host, port, duration):
    """在持续 duration 秒内接收原始串口流，返回字节数与各家族通过 CRC 的帧数。"""
    parser = RawFrames()
    total = 0
    started = time.monotonic()
    with socket.create_connection((host, port), timeout=5) as sock:  # 建连 5 秒超时
        sock.settimeout(3)  # 接收 3 秒无数据即视为异常并抛错
        while time.monotonic() - started < duration:
            data = sock.recv(8191)
            if not data:
                raise EOFError('Raw serial stream closed; check serial device and other clients')
            total += len(data)
            parser.feed(data)
    # 通过标准：必须出现过至少一帧 AA58 心跳，且全程没有 CRC 错误。
    if not parser.frames.get('58') or parser.crc_errors:
        raise ValueError(f'No valid AA58 heartbeat, or CRC errors: {parser.frames}, {parser.crc_errors}')
    return {'bytes': total, 'crc_valid_frames_by_family': parser.frames,
            'crc_errors': parser.crc_errors, 'elapsed_s': time.monotonic() - started}


def camera_session(host, port, duration):
    """在 duration 秒内连续解码相机帧，校验序号与源时间戳严格递增，返回帧率等统计。"""
    sequences, stamps, receipts, sizes = [], [], [], []
    started = time.monotonic()
    with socket.create_connection((host, port), timeout=5) as sock:
        sock.settimeout(3)
        while time.monotonic() - started < duration:
            seq, stamp, width, height, size = camera_frame(sock)
            # 序号与源端时间戳都必须严格递增，否则说明流被截断或复用了旧帧。
            if sequences and (seq <= sequences[-1] or stamp <= stamps[-1]):
                raise ValueError('Camera sequence or source receipt time did not increase')
            sequences.append(seq)
            stamps.append(stamp)
            receipts.append(time.monotonic())  # 本机接收时刻，用于算实际帧率
            sizes.append(size)
    if len(sequences) < 2:
        raise ValueError('Fewer than two decoded camera frames')  # 少于两帧无法判断递增与帧率
    return {'frames': len(sequences), 'first_sequence': sequences[0], 'last_sequence': sequences[-1],
            'width': width, 'height': height, 'eye_width': width // 2,
            'receive_fps': (len(receipts) - 1) / (receipts[-1] - receipts[0]),
            'average_jpeg_bytes': sum(sizes) / len(sizes), 'elapsed_s': time.monotonic() - started}


def probe(host, raw_port, camera_port, duration):
    """跑两轮会话：先按 duration 并发接收，再各重连 2 秒；返回可直接落盘的 JSON 报告。"""
    report = {'host': host, 'raw_port': raw_port, 'camera_port': camera_port,
              'tested_at': time.strftime('%Y-%m-%dT%H:%M:%S%z'), 'commands_sent': 0,  # 恒为 0，声明本测试只收不发
              'scope': 'Concurrent reception and reconnect; no service restart or reboot test',
              'sessions': {}}
    for phase, seconds in (('concurrent', duration), ('reconnect', 2)):
        # Allow the gateway to observe EOF before opening its next exclusive session.
        if phase == 'reconnect':
            time.sleep(0.3)  # 等网关看到上一会话的 EOF，避免独占连接被拒
        with concurrent.futures.ThreadPoolExecutor(max_workers=2) as pool:
            futures = {'raw': pool.submit(raw_session, host, raw_port, seconds),
                       'camera': pool.submit(camera_session, host, camera_port, seconds)}
            for name, future in futures.items():
                try:
                    report['sessions'][f'{phase}_{name}'] = {'result': 'PASS', **future.result()}
                except Exception as error:
                    report['sessions'][f'{phase}_{name}'] = {'result': 'FAIL', 'error': str(error)}
    report['result'] = 'PASS' if all(s['result'] == 'PASS' for s in report['sessions'].values()) else 'FAIL'
    return report


def main():
    """解析参数、执行验收并写出 JSON 报告；退出码 0 = PASS、1 = FAIL、2 = 参数非法。"""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--host', default='192.168.20.70')
    parser.add_argument('--raw-port', type=int, default=9000)  # 原始串口转发端口
    parser.add_argument('--camera-port', type=int, default=9001)  # 相机 JPEG 转发端口
    parser.add_argument('--duration', type=float, default=10)
    parser.add_argument('--output', type=Path, default=Path('artifacts/tcp-repair/lan-acceptance.json'))
    args = parser.parse_args()
    if not 2 <= args.duration <= 300 or not all(1 <= p <= 65535 for p in (args.raw_port, args.camera_port)):
        parser.error('duration must be 2..300 seconds and ports 1..65535')  # argparse 以退出码 2 结束
    report = probe(args.host, args.raw_port, args.camera_port, args.duration)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding='utf-8')
    print(json.dumps(report, ensure_ascii=False, indent=2))
    return 0 if report['result'] == 'PASS' else 1


if __name__ == '__main__':
    raise SystemExit(main())
