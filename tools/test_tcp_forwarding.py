"""Read-only LAN acceptance test; test tooling, not robot-side transport.

Runs the two ports concurrently and reconnects both. Requires Pillow on client.
Never sends bytes to the robot. A PASS proves receipt/format, not motor motion.
"""
import argparse
import concurrent.futures
import io
import json
from pathlib import Path
import socket
import struct
import time


def crc8(data):
    crc = 0
    for value in data:
        crc ^= value
        for _ in range(8):
            crc = ((crc << 1) ^ (7 if crc & 128 else 0)) & 255
    return crc


def crc16(data):
    crc = 65535
    for value in data:
        crc ^= value << 8
        for _ in range(8):
            crc = ((crc << 1) ^ (0x1021 if crc & 32768 else 0)) & 65535
    return crc


class RawFrames:
    def __init__(self):
        self.buffer = bytearray()
        self.frames = {}
        self.crc_errors = 0

    def feed(self, data):
        self.buffer.extend(data)
        while len(self.buffer) >= 3:
            b = self.buffer
            if b[0] != 0xAA or b[1] not in (0x55, 0x58, 0x59, 0x5B):
                del b[0]
                continue
            family = b[1]
            if family == 0x55:
                if not 8 <= b[2] <= 76:
                    del b[0]
                    continue
                size = b[2] + 6
            else:
                if len(b) < 16:
                    break
                size = int.from_bytes(b[10:12], 'little') + 20
                if size > 4116:
                    del b[0]
                    continue
            if len(b) < size:
                break
            packet = bytes(b[:size])
            if packet[-2:] != bytes((family, 0xAA)):
                del b[0]
                continue
            valid = (crc8(packet[2:-3]) == packet[-3] if family == 0x55 else
                     crc16(packet[1:-4]) == int.from_bytes(packet[-4:-2], 'little'))
            if not valid:
                self.crc_errors += 1
                del b[0]
                continue
            if family == 0x58 and (packet[2:6] != b'\x01\x01\x00\x00' or size not in (20, 25)):
                del b[0]
                continue
            self.frames[f'{family:02X}'] = self.frames.get(f'{family:02X}', 0) + 1
            del b[:size]


def exact(sock, size):
    data = bytearray()
    while len(data) < size:
        part = sock.recv(min(8191, size - len(data)))
        if not part:
            raise EOFError('Stream closed before a complete frame')
        data.extend(part)
    return bytes(data)


def camera_frame(sock):
    from PIL import Image
    h = struct.unpack('!4sHHQQIIII', exact(sock, 40))
    magic, version, flags, sequence, stamp, width, height, size, reserved = h
    if (magic != b'UWSC' or version != 1 or flags != 1 or reserved != 0 or
            not 2 <= width <= 8192 or width % 2 or not 1 <= height <= 4096 or
            not 0 < size <= 8 * 1024 * 1024):
        raise ValueError('Invalid UWSC v1 header')
    jpeg = exact(sock, size)
    with Image.open(io.BytesIO(jpeg)) as image:
        if image.format != 'JPEG' or image.size != (width, height):
            raise ValueError('JPEG format/size does not match header')
        image.load()
    return sequence, stamp, width, height, size


def raw_session(host, port, duration):
    parser = RawFrames()
    total = 0
    started = time.monotonic()
    with socket.create_connection((host, port), timeout=5) as sock:
        sock.settimeout(3)
        while time.monotonic() - started < duration:
            data = sock.recv(8191)
            if not data:
                raise EOFError('Raw serial stream closed; check serial device and other clients')
            total += len(data)
            parser.feed(data)
    if not parser.frames.get('58') or parser.crc_errors:
        raise ValueError(f'No valid AA58 heartbeat, or CRC errors: {parser.frames}, {parser.crc_errors}')
    return {'bytes': total, 'crc_valid_frames_by_family': parser.frames,
            'crc_errors': parser.crc_errors, 'elapsed_s': time.monotonic() - started}


def camera_session(host, port, duration):
    sequences, stamps, receipts, sizes = [], [], [], []
    started = time.monotonic()
    with socket.create_connection((host, port), timeout=5) as sock:
        sock.settimeout(3)
        while time.monotonic() - started < duration:
            seq, stamp, width, height, size = camera_frame(sock)
            if sequences and (seq <= sequences[-1] or stamp <= stamps[-1]):
                raise ValueError('Camera sequence or source receipt time did not increase')
            sequences.append(seq)
            stamps.append(stamp)
            receipts.append(time.monotonic())
            sizes.append(size)
    if len(sequences) < 2:
        raise ValueError('Fewer than two decoded camera frames')
    return {'frames': len(sequences), 'first_sequence': sequences[0], 'last_sequence': sequences[-1],
            'width': width, 'height': height, 'eye_width': width // 2,
            'receive_fps': (len(receipts) - 1) / (receipts[-1] - receipts[0]),
            'average_jpeg_bytes': sum(sizes) / len(sizes), 'elapsed_s': time.monotonic() - started}


def probe(host, raw_port, camera_port, duration):
    report = {'host': host, 'raw_port': raw_port, 'camera_port': camera_port,
              'tested_at': time.strftime('%Y-%m-%dT%H:%M:%S%z'), 'commands_sent': 0,
              'scope': 'Concurrent reception and reconnect; no service restart or reboot test',
              'sessions': {}}
    for phase, seconds in (('concurrent', duration), ('reconnect', 2)):
        # Allow the gateway to observe EOF before opening its next exclusive session.
        if phase == 'reconnect':
            time.sleep(0.3)
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
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--host', default='192.168.20.70')
    parser.add_argument('--raw-port', type=int, default=9000)
    parser.add_argument('--camera-port', type=int, default=9001)
    parser.add_argument('--duration', type=float, default=10)
    parser.add_argument('--output', type=Path, default=Path('artifacts/tcp-repair/lan-acceptance.json'))
    args = parser.parse_args()
    if not 2 <= args.duration <= 300 or not all(1 <= p <= 65535 for p in (args.raw_port, args.camera_port)):
        parser.error('duration must be 2..300 seconds and ports 1..65535')
    report = probe(args.host, args.raw_port, args.camera_port, args.duration)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding='utf-8')
    print(json.dumps(report, ensure_ascii=False, indent=2))
    return 0 if report['result'] == 'PASS' else 1


if __name__ == '__main__':
    raise SystemExit(main())
