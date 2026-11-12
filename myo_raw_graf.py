from __future__ import print_function

import enum
import re
from struct import pack, unpack
import sys
import time
import threading

# Python 2/3 互換の queue
try:
    import queue
except ImportError:
    import Queue as queue

import serial
from serial.tools.list_ports import comports

try:
    import pygame
    from pygame.locals import *
    HAVE_PYGAME = True
except ImportError:
    HAVE_PYGAME = False
    print("Pygame is not installed. Drawing will be skipped.")


def multichr(ords):
    return bytes(ords)


def multiord(b):
    return list(b)


class Packet(object):
    def __init__(self, ords):
        self.typ = ords[0]
        self.cls = ords[2]
        self.cmd = ords[3]
        self.payload = multichr(ords[4:])

    def __repr__(self):
        return 'Packet(%02X, %02X, %02X, [%s])' % (
            self.typ,
            self.cls,
            self.cmd,
            ' '.join('%02X' % b for b in multiord(self.payload)),
        )


class BT(object):
    def __init__(self, tty):
        self.ser = serial.Serial(port=tty, baudrate=9600, dsrdtr=1)
        self.buf = []
        self.handlers = []

    def recv_packet(self, timeout=None):
        t0 = time.time()
        self.ser.timeout = None
        while timeout is None or time.time() < t0 + timeout:
            if timeout is not None:
                self.ser.timeout = max(0, t0 + timeout - time.time())
            c = self.ser.read(1)
            if not c:
                return None
            ret = self.proc_byte(ord(c))
            if ret:
                if ret.typ == 0x80:
                    self.handle_event(ret)
                return ret

    def proc_byte(self, c):
        if not self.buf:
            if c in [0x00, 0x80, 0x08, 0x88]:
                self.buf.append(c)
            return None
        elif len(self.buf) == 1:
            self.buf.append(c)
            self.packet_len = 4 + (self.buf[0] & 0x07) + self.buf[1]
            return None
        else:
            self.buf.append(c)

        if self.packet_len and len(self.buf) == self.packet_len:
            p = Packet(self.buf)
            self.buf = []
            return p
        return None

    def handle_event(self, p):
        for h in self.handlers:
            h(p)

    def add_handler(self, h):
        self.handlers.append(h)

    def remove_handler(self, h):
        try:
            self.handlers.remove(h)
        except ValueError:
            pass

    def wait_event(self, cls, cmd, timeout=5.0):
        res = [None]
        def h(p):
            if p.cls == cls and p.cmd == cmd:
                res[0] = p
        self.add_handler(h)
        t0 = time.time()
        while res[0] is None:
            if time.time() - t0 > timeout:
                self.remove_handler(h)
                return None
            self.recv_packet(timeout=1.0)
        self.remove_handler(h)
        return res[0]

    def connect(self, addr, addr_type=1):
        return self.send_command(
            6, 3, pack('<6sBHHHH', multichr(addr), addr_type, 6, 6, 64, 0)
        )

    def discover(self):
        return self.send_command(6, 2, b'\x01')

    def end_scan(self):
        return self.send_command(6, 4)

    def disconnect(self, h):
        return self.send_command(3, 0, pack('<B', h))

    def read_attr(self, con, attr):
        self.send_command(4, 4, pack('<BH', con, attr))
        return self.wait_event(4, 5)

    def write_attr(self, con, attr, val):
        self.send_command(4, 5, pack('<BHB', con, attr, len(val)) + val)
        return self.wait_event(4, 1)

    def send_command(self, cls, cmd, payload=b''):
        s = pack('<4B', 0, len(payload), cls, cmd) + payload
        self.ser.write(s)
        while True:
            p = self.recv_packet()
            if p and p.typ == 0:
                return p
            if p:
                self.handle_event(p)


class MyoRaw(object):
    def __init__(self, tty=None):
        if tty is None:
            tty = self.detect_tty()
        if tty is None:
            raise ValueError('Myo dongle not found!')
        self.bt = BT(tty)
        self.conn = None
        self.emg_handlers = []
        self.pose_handlers = []

    def detect_tty(self):
        for p in comports():
            if re.search(r'PID=2458:0*1', p[2]):
                print('Using device:', p[0])
                return p[0]
        return None

    def run(self, timeout=None):
        self.bt.recv_packet(timeout)

    def connect(self):
        for h in range(3):
            try:
                self.bt.disconnect(h)
            except Exception:
                pass
        try:
            self.bt.end_scan()
        except Exception:
            pass

        print('Scanning for Myo... (Shake your Myo)')
        self.bt.discover()
        addr = None
        target_addr_type = 1

        while True:
            p = self.bt.recv_packet(timeout=1.0)
            if p and p.cls == 6 and p.cmd == 0:
                payload = p.payload
                if (b'Myo' in payload) or (b'\x42\x48\x12\x4A' in payload):
                    addr = list(multiord(p.payload[2:8]))
                    if len(payload) > 8:
                        target_addr_type = multiord(payload[8:9])[0]
                    mac_str = ':'.join('%02X' % b for b in reversed(addr))
                    print(f'Detected Myo! MAC: {mac_str}')
                    break

        self.bt.end_scan()
        print('Connecting...')
        conn_pkt = self.bt.connect(addr, addr_type=target_addr_type)

        if len(conn_pkt.payload) >= 2:
            res_code = unpack('<H', conn_pkt.payload[:2])[0]
            if res_code != 0:
                print(f'Connection rejected. Error code: 0x{res_code:04X}')
                return False

        self.conn = multiord(conn_pkt.payload)[-1]
        evt = self.bt.wait_event(3, 0, timeout=10.0)
        if evt is None:
            print('Connection timeout.')
            return False

        print('Connected successfully!')
        fw = self.read_attr(0x17)
        _, _, _, _, v0, v1, v2, v3 = unpack('<BHBBHHHH', fw.payload)
        print('Firmware: %d.%d.%d.%d' % (v0, v1, v2, v3))

        self.write_attr(0x1D, b'\x01\x00')
        self.write_attr(0x24, b'\x02\x00')
        self.write_attr(0x28, b'\x01\x00')
        self.write_attr(0x19, b'\x01\x03\x01\x01\x00')
        self.write_attr(0x19, b'\x01\x03\x01\x01\x01')

        def handle_data(p):
            if (p.cls, p.cmd) != (4, 5):
                return
            c, attr, typ = unpack('<BHB', p.payload[:4])
            pay = p.payload[5:]

            if attr == 0x27:
                vals = unpack('<8HB', pay)
                emg = vals[:8]
                moving = vals[8]
                for h in self.emg_handlers:
                    h(emg, moving)
            elif attr == 0x23:
                try:
                    if len(pay) >= 3:
                        typ_pose, val_pose, xdir = unpack('<3B', pay[:3])
                        if typ_pose == 3:
                            for h in self.pose_handlers:
                                h(val_pose)
                except Exception:
                    pass

        self.bt.add_handler(handle_data)
        return True

    def disconnect(self):
        if self.conn is not None:
            self.bt.disconnect(self.conn)

    def write_attr(self, attr, val):
        if self.conn is not None:
            self.bt.write_attr(self.conn, attr, val)

    def vibrate(self, length):
        if length in range(1, 4):
            self.write_attr(0x19, pack('<3B', 3, 1, length))

    def add_emg_handler(self, h):
        self.emg_handlers.append(h)

    def add_pose_handler(self, h):
        self.pose_handlers.append(h)


if __name__ == '__main__':
    # 描画用のキューと同期フラグ
    data_queue = queue.Queue(maxsize=100)
    stop_flag = threading.Event()

    # Pygame ウィンドウ設定
    if HAVE_PYGAME:
        pygame.init()
        w, h = 800, 600
        scr = pygame.display.set_mode((w, h))
        pygame.display.set_caption("Myo EMG Oscilloscope (Smooth)")
        clock = pygame.time.Clock()

    last_vals = None

    def plot_batch(scr, vals_list):
        """キューに溜まった複数のデータを一度に描画して追いつく"""
        global last_vals
        D = 5
        DRAW_LINES = True

        for vals in vals_list:
            if last_vals is None:
                last_vals = vals
                continue

            scr.scroll(-D, 0)
            scr.fill((0, 0, 0), (w - D, 0, w, h))

            for i, (u, v) in enumerate(zip(last_vals, vals)):
                if DRAW_LINES:
                    pygame.draw.line(scr, (0, 255, 0),
                                     (w - D, int(h / 8 * (i + 1 - u))),
                                     (w, int(h / 8 * (i + 1 - v))))
                    pygame.draw.line(scr, (50, 50, 50),
                                     (w - D, int(h / 8 * (i + 1))),
                                     (w, int(h / 8 * (i + 1))))
            last_vals = vals


    # --- 通信専用のバックグラウンドスレッド ---
    def myo_worker():
        m = MyoRaw()

        def proc_emg(emg, moving):
            # データをキューに渡す（描画は行わない）
            if not data_queue.full():
                data_queue.put([e / 2000.0 for e in emg])

        m.add_emg_handler(proc_emg)

        def on_pose(pose_val):
            print(f"Pose detected: {pose_val}")
            if pose_val == 1:
                m.vibrate(1)

        m.add_pose_handler(on_pose)

        if m.connect():
            print('\n通信スレッド起動完了。データ受信を開始しました。')
            try:
                while not stop_flag.is_set():
                    m.run(0.1)
            except Exception as e:
                print("通信エラー:", e)
            finally:
                m.disconnect()
                print('\nMyo との通信を切断しました。')

    # スレッドの起動
    worker_thread = threading.Thread(target=myo_worker)
    worker_thread.daemon = True
    worker_thread.start()

    # --- メインスレッド（描画とイベント処理専用） ---
    if HAVE_PYGAME:
        print('Pygame window opened. Press Q or close window to exit.')
        try:
            while True:
                # 60fpsで描画ループを回す
                clock.tick(60)

                # Pygame のキー・ウィンドウイベント処理
                for ev in pygame.event.get():
                    if ev.type == QUIT or (ev.type == KEYDOWN and ev.unicode == 'q'):
                        raise KeyboardInterrupt()

                # キューに溜まったデータをすべて取り出して一気に描画
                batch = []
                while not data_queue.empty():
                    batch.append(data_queue.get_nowait())
                
                if batch:
                    plot_batch(scr, batch)
                    pygame.display.flip()

        except KeyboardInterrupt:
            print("終了処理を開始します...")
        finally:
            # スレッドを安全に停止
            stop_flag.set()
            worker_thread.join()
            pygame.quit()