"""Loopback fixtures validate the acceptance client; no hardware evidence."""
import io
import socket
import struct
import threading
import time
import unittest

from PIL import Image
import test_tcp_forwarding as client

HEARTBEAT = bytes.fromhex('aa5801010000150000000500b873f2b30000000000da2158aa')
CAN = bytes.fromhex('aa55183b0101020000020c00000000f300bb4500033600c80631003f55aa')
INFO = bytes.fromhex('aa5b010101017856341200000000000052d65baa')


def jpeg_bytes():
    out = io.BytesIO()
    Image.new('RGB', (64, 32), (20, 80, 140)).save(out, format='JPEG')
    return out.getvalue()


def packet(jpeg, seq=1, width=64, **fields):
    return struct.pack('!4sHHQQIIII', fields.get('magic', b'UWSC'), fields.get('version', 1),
                       1, seq, seq * 1000000, width, 32, fields.get('size', len(jpeg)), 0) + jpeg


class FixtureServer:
    def __init__(self, payload):
        self.payload = payload
        self.stop = threading.Event()
        self.listener = socket.socket()
        self.listener.bind(('127.0.0.1', 0))
        self.listener.listen(4)
        self.listener.settimeout(0.1)
        self.port = self.listener.getsockname()[1]
        self.connections = 0
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
                        for offset in range(0, len(data), 17):
                            conn.sendall(data[offset:offset + 17])
                        try:
                            data = conn.recv(4096)
                            if not data:
                                break
                            self.received.extend(data)
                        except socket.timeout:
                            pass
                        self.stop.wait(0.025)
                    except OSError:
                        break
        self.listener.close()

    def close(self):
        self.stop.set()
        self.thread.join(timeout=2)
        if self.thread.is_alive():
            raise RuntimeError('Fixture did not stop')


class AcceptanceClientTests(unittest.TestCase):
    def test_known_protocol_samples_all_fragment_boundaries(self):
        for sample, family in ((HEARTBEAT, '58'), (CAN, '55'), (INFO, '5B')):
            for cut in range(len(sample) + 1):
                parser = client.RawFrames()
                parser.feed(sample[:cut])
                parser.feed(sample[cut:])
                self.assertEqual(parser.frames, {family: 1})
                self.assertEqual(parser.crc_errors, 0)

    def test_crc_corruption_detected_and_next_frame_recovered(self):
        corrupt = bytearray(HEARTBEAT)
        corrupt[16] ^= 1
        parser = client.RawFrames()
        parser.feed(corrupt + HEARTBEAT)
        self.assertEqual(parser.crc_errors, 1)
        self.assertEqual(parser.frames, {'58': 1})

    def decode(self, data):
        a, b = socket.socketpair()
        with a, b:
            a.sendall(data)
            a.shutdown(socket.SHUT_WR)
            return client.camera_frame(b)

    def test_camera_header_and_decoded_dimensions(self):
        self.assertEqual(self.decode(packet(jpeg_bytes()))[:4], (1, 1000000, 64, 32))

    def test_camera_invalid_headers_rejected_before_payload_read(self):
        for fields in ({'magic': b'HTTP'}, {'version': 2}, {'size': 8 * 1024 * 1024 + 1}, {'size': 0}):
            with self.subTest(fields=fields), self.assertRaises(ValueError):
                self.decode(packet(b'', **fields))

    def test_camera_jpeg_corruption_and_mismatched_dimensions(self):
        with self.assertRaises(Exception):
            self.decode(packet(b'not a JPEG'))
        with self.assertRaises(ValueError):
            self.decode(packet(jpeg_bytes(), width=66))
        with self.assertRaises(EOFError):
            self.decode(packet(jpeg_bytes())[:-10])

    def test_concurrent_streams_and_reconnect_send_no_commands(self):
        jpeg = jpeg_bytes()
        raw = FixtureServer(lambda _: HEARTBEAT + CAN + INFO)
        camera = FixtureServer(lambda seq: packet(jpeg, seq))
        try:
            report = client.probe('127.0.0.1', raw.port, camera.port, 2)
            self.assertEqual(report['result'], 'PASS', report)
            self.assertEqual(len(report['sessions']), 4)
            self.assertEqual(raw.connections, 2)
            self.assertEqual(camera.connections, 2)
            self.assertFalse(raw.received or camera.received)
        finally:
            raw.close()
            camera.close()


if __name__ == '__main__':
    unittest.main(verbosity=2)
