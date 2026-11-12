import time
import multiprocessing as mp
import numpy as np
import zmq


# ============================================================
# Apple Vision Pro 参数
# ============================================================

# 启动及重连时依次尝试候选IP；若已固定IP，可只保留一个地址。
AVP_IP_CANDIDATES = ("192.168.9.109", "192.168.9.117")
AVP_DISCOVERY_TIMEOUT = 8.0
AVP_DISCOVERY_INTERVAL = 2.0

# Python PUB绑定，C++ SUB连接。单帧为35个小端float64，共280字节。
# 前32个数是左右手腕矩阵（按行），随后为左右捏合和connected。
ZMQ_ENDPOINT = "tcp://127.0.0.1:5557"

# 超过该时间没有收到新的 AVP tracking 数据，
# 判定 Vision Pro 数据断流
STALE_TIMEOUT = 0.2
RECONNECT_TIMEOUT = 2.0  # 持续断流2秒后重建SDK连接


# 距离单位：米。滞环阈值与连续新帧确认共同抑制误触发。
PINCH_ON = 0.015       # 小于1.5 cm持续80 ms：捏合
PINCH_OFF = 0.030      # 大于3 cm持续50 ms：松开
PINCH_ON_TIME = 0.080
PINCH_OFF_TIME = 0.050


class PinchDetector:
    def __init__(self):
        self.reset()

    def reset(self):
        self.state = False
        self.pending = None
        self.since = 0.0
        self.last_sample = None

    def update(self, distance, now):
        # 只对真正的新tracking帧调用；断流前的候选确认不延续。
        if not np.isfinite(distance) or distance < 0:
            self.reset()
            return False
        if self.last_sample is not None and now - self.last_sample > STALE_TIMEOUT:
            self.reset()
        self.last_sample = now
        desired = self.state
        if not self.state and distance < PINCH_ON:
            desired = True
        elif self.state and distance > PINCH_OFF:
            desired = False
        if desired == self.state:
            self.pending = None
        elif self.pending != desired:
            self.pending = desired
            self.since = now
        elif now - self.since >= (PINCH_ON_TIME if desired else PINCH_OFF_TIME):
            self.state = desired
            self.pending = None
        return self.state


def _receive_avp(ip, writer):
    """SDK仅在子进程运行；重连时由主进程回收整个旧连接。"""
    import os
    import signal

    signal.signal(signal.SIGINT, signal.SIG_IGN)
    try:
        # 包括SDK后台线程和原生库日志，避免摘戴期间反复刷屏。
        with open(os.devnull, "w") as quiet:
            os.dup2(quiet.fileno(), 1)
            os.dup2(quiet.fileno(), 2)
            from avp_stream import VisionProStreamer

            streamer = VisionProStreamer(ip=ip, record=True)
            previous_count = 0
            while True:
                count = len(streamer.recording)
                if count > previous_count:
                    previous_count = count
                    sample = streamer.latest
                    if sample is not None:
                        try:
                            values = np.concatenate((
                                np.asarray(sample["left_wrist"], dtype=float).reshape(16),
                                np.asarray(sample["right_wrist"], dtype=float).reshape(16),
                                [float(sample["left_pinch_distance"]),
                                 float(sample["right_pinch_distance"])],
                            ))
                            if np.isfinite(values).all() and (values[32:] >= 0).all():
                                writer.send((time.monotonic(), values.astype("<f8").tobytes()))
                        except (KeyError, ValueError, TypeError):
                            pass
                time.sleep(0.001)
    except (Exception, KeyboardInterrupt):
        pass
    finally:
        writer.close()
        # 不等待SDK可能遗留的非daemon线程；OS回收该进程的连接。
        os._exit(0)


def _stop_worker(process, reader):
    if process is not None:
        if process.is_alive():
            process.terminate()
        process.join(timeout=0.2)
        if process.is_alive():
            process.kill()
            process.join(timeout=0.2)
        if not process.is_alive():
            process.close()
    if reader is not None:
        reader.close()


def main():
    import signal

    stop_requested = False

    def request_stop(signum, frame):
        nonlocal stop_requested
        stop_requested = True

    old_int = signal.signal(signal.SIGINT, request_stop)
    old_term = signal.signal(signal.SIGTERM, request_stop)
    ctx = mp.get_context("spawn")
    context = zmq.Context()
    publisher = context.socket(zmq.PUB)
    publisher.setsockopt(zmq.LINGER, 0)
    publisher.setsockopt(zmq.SNDHWM, 1)
    process = reader = None
    left_detector, right_detector = PinchDetector(), PinchDetector()
    packet = np.zeros(35, dtype="<f8")
    packet[:16] = np.eye(4).reshape(16)
    packet[16:32] = np.eye(4).reshape(16)
    connected = False
    last_frame_time = None
    attempt_start = 0.0
    next_attempt = 0.0
    preferred_ip = None
    ip_index = 0
    current_ip = None

    try:
        publisher.bind(ZMQ_ENDPOINT)
        print("[AVP] 等待连接；摘戴后自动重连，Ctrl+C退出。", flush=True)
        while not stop_requested:
            now = time.monotonic()
            if process is None and now >= next_attempt:
                # 优先重连上一次成功的IP；失败后遍历其余候选地址。
                addresses = list(dict.fromkeys(
                    ([preferred_ip] if preferred_ip else []) + list(AVP_IP_CANDIDATES)
                ))
                if not addresses:
                    raise ValueError("AVP_IP_CANDIDATES不能为空")
                current_ip = addresses[ip_index % len(addresses)]
                ip_index += 1
                reader, writer = ctx.Pipe(duplex=False)
                process = ctx.Process(target=_receive_avp,
                                      args=(current_ip, writer), daemon=True)
                try:
                    process.start()
                except Exception:
                    reader.close()
                    process = reader = None
                    raise
                finally:
                    writer.close()
                attempt_start = time.monotonic()
                last_frame_time = None

            worker_closed = False
            if reader is not None:
                # 有界排空，避免生产者持续发送导致主循环无法响应退出。
                for _ in range(64):
                    if not reader.poll():
                        break
                    try:
                        stamp, payload = reader.recv()
                    except (EOFError, OSError):
                        worker_closed = True
                        break
                    now = time.monotonic()
                    if now - stamp > STALE_TIMEOUT or stamp > now:
                        continue
                    if last_frame_time is not None and stamp <= last_frame_time:
                        continue
                    values = np.frombuffer(payload, dtype="<f8")
                    if values.size != 34 or not np.isfinite(values).all():
                        continue
                    last_frame_time = stamp
                    packet[:32] = values[:32]
                    packet[32] = left_detector.update(values[32], stamp)
                    packet[33] = right_detector.update(values[33], stamp)

            now = time.monotonic()
            fresh = (last_frame_time is not None and
                     now - last_frame_time <= STALE_TIMEOUT and
                     not worker_closed and process is not None and process.is_alive())
            if fresh != connected:
                connected = fresh
                if connected:
                    preferred_ip = current_ip
                    print("[AVP] 已连接：" + current_ip, flush=True)
                else:
                    print("[AVP] 已断流，等待自动重连。", flush=True)
            if not connected:
                left_detector.reset()
                right_detector.reset()
                packet[32:34] = 0.0
            packet[34] = float(connected)
            publisher.send(packet.tobytes())

            if process is not None:
                timed_out = (
                    now - attempt_start >= AVP_DISCOVERY_TIMEOUT
                    if last_frame_time is None
                    else now - last_frame_time >= RECONNECT_TIMEOUT
                )
                if worker_closed or not process.is_alive() or timed_out:
                    # 先发布断开状态，再回收SDK，不能用旧帧伪装连接。
                    connected = False
                    packet[32:] = 0.0
                    publisher.send(packet.tobytes())
                    left_detector.reset()
                    right_detector.reset()
                    if last_frame_time is not None:
                        ip_index = 0
                    _stop_worker(process, reader)
                    process = reader = None
                    last_frame_time = None
                    next_attempt = time.monotonic() + AVP_DISCOVERY_INTERVAL
            time.sleep(0.001)
    finally:
        # 最后一帧仅作为补充；C++仍必须保留接收超时检测。
        packet[32:] = 0.0
        try:
            publisher.send(packet.tobytes(), flags=zmq.DONTWAIT)
        except zmq.ZMQError:
            pass
        _stop_worker(process, reader)
        publisher.close(linger=0)
        context.term()
        signal.signal(signal.SIGINT, old_int)
        signal.signal(signal.SIGTERM, old_term)
        print("[AVP] 已退出。", flush=True)


if __name__ == "__main__":
    main()
