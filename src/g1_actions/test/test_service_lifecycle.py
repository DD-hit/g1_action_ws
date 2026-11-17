"""Linux process ownership regression tests; no camera, ROS or robot commands."""
from pathlib import Path
import os
import signal
import subprocess
import sys
import tempfile
import time
import unittest

ROOT=Path(__file__).resolve().parents[1]
MANAGER=ROOT/'scripts/managed_service.py'


class ServiceLifecycle(unittest.TestCase):
    def setUp(self):
        self.directory=tempfile.TemporaryDirectory(prefix='ele_process_test_')
        self.base=Path(self.directory.name)
        self.env=dict(os.environ,ELE_SERVICE_DIR=str(self.base/'state'),ROS_DOMAIN_ID='198')
        self.runners=[];self.logs=[]

    def tearDown(self):
        for process in self.runners:
            if process.poll() is None:
                process.send_signal(signal.SIGINT)
                process.wait(timeout=12)
        for log in self.logs:log.close()
        self.directory.cleanup()

    def launch(self,body):
        log=(self.base/('log'+str(len(self.logs)))).open('w');self.logs.append(log)
        process=subprocess.Popen([sys.executable,str(MANAGER),'vision','--',sys.executable,'-c',body],
                                 env=self.env,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
        self.runners.append(process)
        return process

    def wait_file(self,path):
        until=time.monotonic()+4
        while time.monotonic()<until:
            if path.exists():return
            time.sleep(.03)
        self.fail('Worker did not start: '+''.join((self.base/f'log{i}').read_text() for i in range(len(self.logs))))

    def detached_worker(self,exit_parent=False):
        marker=self.base/'grandchild.pid'
        grandchild=("import os,signal,time;from pathlib import Path;"
                    "signal.signal(signal.SIGINT,signal.SIG_IGN);"
                    "signal.signal(signal.SIGTERM,signal.SIG_IGN);"
                    f"Path({str(marker)!r}).write_text(str(os.getpid()));time.sleep(60)")
        body=("import subprocess,sys,time,signal;from pathlib import Path;"
              f"subprocess.Popen([sys.executable,'-c',{grandchild!r}],start_new_session=True);"
              f"p=Path({str(marker)!r});\nwhile not p.exists():time.sleep(.02)\n")
        body+=('sys.exit(7)' if exit_parent else
               'signal.signal(signal.SIGINT,signal.SIG_IGN);signal.signal(signal.SIGTERM,signal.SIG_IGN);time.sleep(60)')
        return self.launch(body),marker

    def test_repeated_interrupt_reaps_detached_grandchild(self):
        process,marker=self.detached_worker()
        self.wait_file(marker);pid=int(marker.read_text())
        process.send_signal(signal.SIGINT);time.sleep(.1);process.send_signal(signal.SIGINT)
        self.assertEqual(process.wait(timeout=12),0)
        self.assertFalse(Path(f'/proc/{pid}').exists(),'Detached grandchild remained')

    def test_parent_exit_still_reaps_orphan(self):
        process,marker=self.detached_worker(exit_parent=True)
        self.wait_file(marker);pid=int(marker.read_text())
        self.assertEqual(process.wait(timeout=12),7)
        self.assertFalse(Path(f'/proc/{pid}').exists(),'Orphan remained after parent exit')

    def test_duplicate_start_does_not_spawn_another_worker(self):
        marker=self.base/'started'
        first=self.launch(f"import time;from pathlib import Path;Path({str(marker)!r}).touch();time.sleep(60)")
        self.wait_file(marker)
        second_marker=self.base/'should_not_exist'
        second=self.launch(f"from pathlib import Path;Path({str(second_marker)!r}).touch()")
        self.assertEqual(second.wait(timeout=3),2)
        self.assertFalse(second_marker.exists())
        self.assertIsNone(first.poll())

    def test_scoped_stop_preserves_unrelated_process(self):
        unrelated=subprocess.Popen([sys.executable,'-c','import time;time.sleep(60)'])
        try:
            marker=self.base/'started'
            managed=self.launch(f"import time;from pathlib import Path;Path({str(marker)!r}).touch();time.sleep(60)")
            self.wait_file(marker)
            body=f"import sys;sys.path.insert(0,{str(ROOT/'scripts')!r});from managed_service import stop_perception;stop_perception(include_legacy=False)"
            result=subprocess.run([sys.executable,'-c',body],env=self.env,capture_output=True,text=True,timeout=12)
            self.assertEqual(result.returncode,0,result.stdout+result.stderr)
            self.assertEqual(managed.wait(timeout=3),0)
            self.assertIsNone(unrelated.poll())
        finally:unrelated.terminate();unrelated.wait(timeout=3)


if __name__=='__main__':unittest.main(verbosity=2)
