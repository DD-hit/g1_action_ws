"""Admission readiness from fresh, sustained vision results; no ROS side effects."""
import math


class CameraReceipts:
    """Raw image reception remains observable while inference is paused."""
    def __init__(self):self.received={}

    def reset(self):self.received.clear()

    def observe(self,health,now):
        if not isinstance(health,dict):return
        for stream in ('color','depth'):
            age=health.get(stream+'_receive_age_s')
            stamp=health.get(stream+'_stamp_age_s')
            if (isinstance(age,(int,float)) and not isinstance(age,bool)
                    and math.isfinite(age) and age>=0
                    and isinstance(stamp,(int,float)) and math.isfinite(stamp)):
                received=now-age
                self.received[stream]=max(received,self.received.get(stream,-math.inf))

    def last_frame(self):
        if len(self.received)!=2:return None
        return min(self.received.values())


class FrameReadiness:
    def __init__(self, max_frame_age=.6, stable_seconds=2., min_frames=6,
                 max_receive_gap=.8):
        self.max_frame_age = float(max_frame_age)
        self.stable_seconds = float(stable_seconds)
        self.min_frames = int(min_frames)
        self.max_receive_gap = float(max_receive_gap)
        if (not all(math.isfinite(v) for v in
                    (self.max_frame_age, self.stable_seconds, self.max_receive_gap))
                or self.max_frame_age <= 0 or self.stable_seconds < 0
                or self.min_frames < 1 or self.max_receive_gap <= 0):
            raise ValueError('Invalid camera readiness limits')
        self.since = self.arrival = self.stamp = self.sequence = None
        self.frames = 0
        self.reason = '等待相机同步 RGB-D 帧'

    def invalidate(self, reason):
        self.since = None
        self.frames = 0
        self.reason = reason

    def observe(self, data, now, wall_now):
        try:
            stamp = float(data['stamp_sec'])
            sequence = data['sequence']
            if not isinstance(sequence, int) or isinstance(sequence, bool):
                raise ValueError('invalid sequence')
        except (KeyError, ValueError, TypeError):
            self.invalidate(data.get('reason') or '视觉消息缺少有效帧序号或时间戳')
            return
        age = wall_now - stamp
        if not math.isfinite(age) or not 0 <= age <= self.max_frame_age:
            self.invalidate('RGB-D 帧已过期或时间戳超前')
            return
        if self.stamp is not None and stamp <= self.stamp:
            self.invalidate('RGB-D 时间戳未前进')
            return
        # A delayed message, missed interval or restarted publisher must qualify
        # again; a single recovery frame never reopens task admission.
        if (self.since is None or self.arrival is None
                or now - self.arrival >= self.max_receive_gap
                or (self.sequence is not None and sequence <= self.sequence)):
            self.since = now
            self.frames = 0
        self.arrival, self.stamp, self.sequence = now, stamp, sequence
        self.frames += 1
        self.reason = ''

    def check(self, now, wall_now):
        if self.reason:
            return False, self.reason
        if self.arrival is None or now - self.arrival >= self.max_receive_gap:
            self.invalidate('有效 RGB-D 结果接收超时')
            return False, self.reason
        # max_frame_age is checked on arrival, just as in the vision pipeline.
        # Do not reclassify normal waiting between low-FPS results as a camera
        # failure. Task target() still validates its original capture timestamp.
        duration = now - self.since
        if duration < self.stable_seconds or self.frames < self.min_frames:
            return False, (f'等待连续稳定 RGB-D：{duration:.1f}/{self.stable_seconds:.1f} 秒，'
                           f'{self.frames}/{self.min_frames} 帧')
        return True, ''

    def diagnostics(self, now, wall_now):
        return {
            'stable_seconds': 0. if self.since is None else max(0., now - self.since),
            'valid_frames': self.frames,
            'result_receive_age_s': None if self.arrival is None else now - self.arrival,
            'result_stamp_age_s': None if self.stamp is None else wall_now - self.stamp,
        }
