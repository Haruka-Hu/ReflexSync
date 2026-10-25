"""Protocol, reliable trace download and analysis helpers for the motor GUI."""
from __future__ import annotations

import math
import zlib
from dataclasses import dataclass, asdict


class LineBuffer:
    """Newline framing across arbitrary USB/BLE fragments; discard overlong lines."""
    def __init__(self, limit: int = 4096):
        self.limit = limit
        self.buffer = bytearray()
        self.discard = False

    def feed(self, data: bytes) -> list[str]:
        lines = []
        for byte in data:
            if byte in (10, 13):
                if self.buffer and not self.discard:
                    lines.append(self.buffer.decode('utf-8', errors='replace'))
                self.buffer.clear()
                self.discard = False
            elif not self.discard:
                if len(self.buffer) >= self.limit:
                    self.buffer.clear()
                    self.discard = True
                else:
                    self.buffer.append(byte)
        return lines


def fields(line: str) -> dict[str, str]:
    return dict(word.split('=', 1) for word in line.split() if '=' in word)


@dataclass(frozen=True)
class TracePoint:
    index: int
    time_us: int
    angle: float
    speed: float
    raw: float
    pwm: float
    drop: float
    baseline: float

    @classmethod
    def parse(cls, line: str) -> 'TracePoint':
        p = fields(line)
        point = cls(int(p['i']), int(p['us']), *(float(p[k]) for k in
                    ('angle', 'speed', 'raw', 'pwm', 'drop', 'baseline')))
        if point.index < 0 or not 0 <= point.time_us <= 0xffffffff or not all(
                math.isfinite(getattr(point, k)) for k in
                ('angle', 'speed', 'raw', 'pwm', 'drop', 'baseline')):
            raise ValueError('轨迹含无效索引、时间或非有限数值')
        return point

    def to_dict(self):
        return asdict(self)


class TraceReplacedError(ValueError):
    pass


def checked_trace_fields(line: str, *, require_crc: bool) -> dict[str, str]:
    payload, separator, checksum = line.rpartition(' crc=')
    if separator:
        if len(checksum) != 8 or zlib.crc32(payload.encode('ascii')) != int(checksum, 16):
            raise ValueError('轨迹 CRC 校验失败')
    elif require_crc:
        raise ValueError('轨迹缺少 CRC')
    words = line.split()
    result = fields(line)
    if len(result) != len(words)-1:
        raise ValueError('轨迹字段缺失、重复或粘连')
    return result


class TraceReceiver:
    """One trace download, with validated chunks and optional V8 request tokens.

    Invalid rows raise without deleting already verified rows. The caller retries
    at TRACE_END or its deadline. Rows outside a validated BEGIN/END are ignored.
    """
    def __init__(self, *, require_crc=False, request_ids=False, shot=-1):
        self.require_crc, self.request_ids = require_crc, request_ids
        self.shot = shot
        self.total = None
        self.points: dict[int, TracePoint] = {}
        self.metadata: dict[str, str] = {}
        self.offset = self.request_id = 0
        self.chunk_open = False

    def request(self, offset: int, request_id: int) -> str:
        self.offset, self.request_id = offset, request_id
        self.chunk_open = False
        return f'TRACE {offset} {request_id}' if self.request_ids else f'TRACE {offset}'

    def feed(self, line: str) -> bool:
        p = checked_trace_fields(line, require_crc=self.require_crc)
        if self.request_ids and int(p['request']) != self.request_id:
            return False  # Delayed response from a canceled/previous request.
        shot = int(p['shot'])
        if not 0 <= shot <= 0xffffffff:
            raise ValueError('轨迹编号无效')
        if line.startswith('TRACE_BEGIN '):
            if int(p['offset']) != self.offset:
                return False
            if self.shot >= 0 and self.shot != shot:
                raise TraceReplacedError('轨迹已被下一次叩击覆盖')
            total, target, window = int(p['total']), float(p['target']), int(p['window_us'])
            if not 0 <= self.offset <= total <= 2048 or not math.isfinite(target) or target < 0 or (total>0 and target==0) or not 0 < window <= 1000000:
                raise ValueError('轨迹头范围无效')
            if self.total is not None and total != self.total:
                raise ValueError('轨迹总数不一致')
            for key in ('retract_angle', 'kp', 'ki', 'ks', 'kv', 'kg'):
                if key in p and not math.isfinite(float(p[key])):
                    raise ValueError('轨迹参数无效')
            # Commit only after the complete header has passed validation.
            self.shot, self.total, self.metadata = shot, total, p
            self.chunk_open = True
        elif self.chunk_open and shot == self.shot:
            end = min(self.total, self.offset+8)
            if line.startswith('TRACE '):
                point = TracePoint.parse(line)
                if not self.offset <= point.index < end:
                    raise ValueError('轨迹索引不属于当前分块')
                if point.index in self.points and self.points[point.index] != point:
                    raise ValueError('重复轨迹点内容不一致')
                self.points[point.index] = point
            elif line.startswith('TRACE_END '):
                if int(p['total']) != self.total or int(p['next']) != end or (
                        'offset' in p and int(p['offset']) != self.offset):
                    raise ValueError('轨迹分块尾不匹配')
                self.chunk_open = False
                return True
        return False


def analyze_trace(points: list[TracePoint], target: float, *, contact: bool,
                  window_us: int = 6000) -> dict:
    """Encoder metrics, not a force measurement.

    Omit the detector lookback and backtrack a long final collision deceleration.
    Settling time requires the final pre-drop in-band run, not an earlier run
    followed by an overshoot. Also report the longest in-band run separately.
    """
    result = {'samples': len(points), 'qualified': False, 'shot_success': False, 'settling_ms': None,
              'stable_ms': 0.0, 'longest_stable_ms': 0.0}
    if not points or not math.isfinite(target) or target <= 0:
        return result
    cutoff = points[-1].time_us - (window_us if contact else 0)
    onset_us = None
    if contact:
        # A compliant target can decelerate the hammer for longer than the
        # detector window. Backtrack the final large drop, using a fixed 90%
        # recent-peak threshold independent of the requested acceptance band.
        # This estimates impact onset; it is not a force-sensor timestamp.
        tail = [p for p in points if p.time_us >= points[-1].time_us-30000]
        peak = max((p.speed for p in tail), default=0)
        if peak >= 40 and points[-1].speed <= peak*.4:
            crossing = next((p for p in reversed(tail[:-1]) if p.speed >= peak*.9), None)
            if crossing is not None:
                onset_us = crossing.time_us
                # Compensate for the configured speed filter's 2ms lag.
                cutoff = min(cutoff, onset_us-2000)
    result['estimated_drop_onset_us'] = onset_us
    usable = [p for p in points if p.time_us < cutoff and math.isfinite(p.speed)]
    if not usable:
        return result
    run_start = None
    longest = 0
    max_gap = max(3000, int(window_us))
    previous = None
    for p in usable:
        if previous is not None and p.time_us - previous > max_gap:
            run_start = None
        if abs(p.speed-target) <= target*.1:
            if run_start is None:
                run_start = p.time_us
            longest = max(longest, p.time_us-run_start)
        else:
            run_start = None
        previous = p.time_us
    stable_us = usable[-1].time_us-run_start if run_start is not None else 0
    steady = [p for p in usable if run_start is not None and p.time_us >= run_start]
    result.update({
        'qualified': stable_us >= 10000,
        'shot_success': contact and stable_us >= 10000,
        'settling_ms': run_start/1000 if run_start is not None and stable_us >= 10000 else None,
        'stable_ms': stable_us/1000,
        'longest_stable_ms': longest/1000,
        'overshoot_pct': max(0.0, (max(p.speed for p in usable)/target-1)*100),
        'rms_error': math.sqrt(sum((p.speed-target)**2 for p in usable)/len(usable)),
        'steady_mean': sum(p.speed for p in steady)/len(steady) if steady else None,
        'steady_rms_error': math.sqrt(sum((p.speed-target)**2 for p in steady)/len(steady)) if steady else None,
        'pre_drop_speed': usable[-1].speed,
        'steady_position_speed': ((steady[-1].angle-steady[0].angle)*1e6/
            (steady[-1].time_us-steady[0].time_us)) if len(steady)>1 else None,
        'cutoff_us': cutoff,
        'max_gap_us': max((b.time_us-a.time_us for a,b in zip(usable,usable[1:])), default=0),
    })
    return result
