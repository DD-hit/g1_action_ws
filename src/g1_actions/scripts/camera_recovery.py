"""Nonblocking perception recovery policy; never commands the robot."""
import threading
import time


def startup_components(camera_present, vision_present):
    """Start missing components without stopping existing publishers."""
    if not camera_present and not vision_present:
        return ('perception',)
    if not camera_present:
        return ('camera',)
    if not vision_present:
        return ('vision',)
    return ()


class CameraRecovery:
    def __init__(self, stop_owned, clock=time.monotonic, stale_s=3., warmup_s=60.):
        self.stop_owned=stop_owned
        self.clock=clock;self.stale_s=stale_s;self.warmup_s=warmup_s
        self.until=clock()+warmup_s
        self.attempts=0;self.thread=None;self.error=None
        self.startup_pending=False

    def request_startup(self):
        """Force one owned restart before initial admission, even if frames exist."""
        self.startup_pending=True
        self.until=self.clock()

    @property
    def active(self):
        return self.thread is not None

    def tick(self, last_frame, ready, safe):
        now=self.clock()
        if self.thread is not None:
            if self.thread.is_alive():return None
            self.thread.join();self.thread=None
            if self.error is not None:
                self.until=now+min(60.,5.*2**min(self.attempts-1,4))
                return 'failed'
            self.until=now+self.warmup_s
            self.startup_pending=False
            return 'start'
        if ready and not self.startup_pending:
            self.attempts=0;self.error=None
            self.until=now
            return None
        if not safe or now<self.until:return None
        if not self.startup_pending and last_frame is not None and now-last_frame<self.stale_s:return None
        self.attempts+=1;self.error=None
        def stop():
            try:self.stop_owned()
            except Exception as error:self.error=str(error)
        self.thread=threading.Thread(target=stop,name='camera-recovery')
        self.thread.start()
        return 'stopping'

    def join(self):
        if self.thread is not None:self.thread.join()
