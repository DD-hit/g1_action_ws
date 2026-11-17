#!/usr/bin/env python3
"""Own camera/vision process trees, including orphaned grandchildren."""
import argparse
import ctypes
import fcntl
import json
import os
from pathlib import Path
import signal
import subprocess
import threading
import time


def processes():
    result={}
    for path in Path('/proc').glob('[0-9]*'):
        try:
            fields=(path/'stat').read_text().rsplit(')',1)[1].split()
            result[int(path.name)]={'ppid':int(fields[1]),'start':fields[19],
                'state':fields[0],'argv':(path/'cmdline').read_bytes().decode().split('\0')}
        except (OSError,ValueError,IndexError,UnicodeError):continue
    return result


def descendants(roots,table):
    owned=set(roots)
    while True:
        more={pid for pid,p in table.items() if p['ppid'] in owned}-owned
        if not more:return owned
        owned.update(more)


def reap(children=()):
    while True:
        try:pid,status=os.waitpid(-1,os.WNOHANG)
        except ChildProcessError:return
        if not pid:return
        for child in children:
            if child.pid==pid:child.returncode=os.waitstatus_to_exitcode(status)


def terminate(roots,children=()):
    table=processes()
    known={pid:table[pid]['start'] for pid in descendants(roots,table) if pid in table}
    top={pid for pid in roots if pid in table}
    for pid in list(top):
        parent=table[pid]['ppid']
        while parent in table:
            if parent in roots:top.discard(pid);break
            parent=table[parent]['ppid']
    for sig,seconds in ((signal.SIGINT,4.),(signal.SIGTERM,2.),(signal.SIGKILL,2.)):
        table=processes()
        live={pid for pid,start in known.items() if pid in table and table[pid]['start']==start and table[pid]['state']!='Z'}
        if not live:break
        for pid in (top & live if sig==signal.SIGINT else live):
            try:os.kill(pid,sig)
            except ProcessLookupError:pass
        until=time.monotonic()+seconds
        while time.monotonic()<until:
            reap(children);table=processes()
            live={pid for pid,start in known.items() if pid in table and table[pid]['start']==start and table[pid]['state']!='Z'}
            for pid in descendants(live,table):
                if pid in table:known.setdefault(pid,table[pid]['start'])
            if not live:break
            time.sleep(.1)
    reap(children);table=processes()
    return [pid for pid,start in known.items() if pid in table and table[pid]['start']==start and table[pid]['state']!='Z']


def same_domain(pid):
    try:
        values=Path(f'/proc/{pid}/environ').read_bytes().split(b'\0')
        actual=next((v.split(b'=',1)[1].decode() for v in values if v.startswith(b'ROS_DOMAIN_ID=')),'0')
        return actual==os.environ.get('ROS_DOMAIN_ID','0')
    except (OSError,UnicodeError):return False


def legacy(role):
    found=[]
    for pid,p in processes().items():
        argv=p['argv']
        if not argv or pid==os.getpid():continue
        camera=(Path(argv[0]).name in ('realsense2_camera_node','ele_realsense_pipeline') and '__ns:=/camera' in argv)
        camera|=all(token in argv for token in ('launch','realsense2_camera','rs_launch.py','camera_namespace:=camera'))
        vision=(any('/g1_actions/lib/g1_actions/elevator_vision' in a for a in argv) or all(token in argv for token in ('run','g1_actions','elevator_vision'))) and same_domain(pid)
        perception=any(a.endswith('/g1_actions/scripts/perception.py') for a in argv) and same_domain(pid)
        if (role=='camera' and camera) or (role=='vision' and vision) or (role=='perception' and (camera or vision or perception)):
            found.append(pid)
    return found


def state_directory():
    path=Path(os.environ.get('ELE_SERVICE_DIR','/run/ele_elevator'))
    path.mkdir(parents=True,exist_ok=True)
    return path


def supervise(role,commands):
    domain=os.environ.get('ROS_DOMAIN_ID','0')
    key=role if role=='camera' else role+'-'+domain
    stopped=threading.Event();children=[];lock=None
    def stopping(signum,frame):stopped.set()
    for sig in (signal.SIGINT,signal.SIGTERM,signal.SIGHUP):signal.signal(sig,stopping)
    # Reparent descendants to this supervisor if ROS launch exits before its child.
    if ctypes.CDLL(None,use_errno=True).prctl(36,1,0,0,0)!=0:
        raise OSError(ctypes.get_errno(),'Cannot enable child subreaper')
    try:
        lock=(state_directory()/(key+'.lock')).open('a+')
        try:fcntl.flock(lock.fileno(),fcntl.LOCK_EX|fcntl.LOCK_NB)
        except BlockingIOError:
            print(f'{role} 已运行；请勿重复启动。需要清理时运行 bash scripts/ele.sh stop-perception。',flush=True)
            lock.close();lock=None
            return 2
        busy=legacy(role)
        if busy:
            print(f'已有相机/视觉进程 {busy}；请先运行 bash scripts/ele.sh stop-perception，再启动 perception。',flush=True)
            return 2
        record={'pid':os.getpid(),'start':processes()[os.getpid()]['start'],'role':role,'domain':domain}
        lock.seek(0);lock.truncate();json.dump(record,lock);lock.flush()
        for command in commands:children.append(subprocess.Popen(command,start_new_session=True))
        while not stopped.wait(.2):
            if any(child.poll() is not None for child in children):
                return next((child.returncode for child in children if child.returncode not in (None,0)),0)
        return 0
    finally:
        # Repeated Ctrl+C sets a flag instead of interrupting cleanup.
        owned=descendants({os.getpid()},processes())-{os.getpid()}
        remaining=terminate(owned,children)
        if remaining:print(f'仍有进程等待内核操作退出：{remaining}',flush=True)
        if lock is not None:lock.seek(0);lock.truncate();lock.close()
        if children:
            print(f'{role} 已退出，所属子进程已回收。' if not remaining else f'{role} 清理未完成。',flush=True)
        if remaining:raise RuntimeError('服务子进程仍在等待内核操作退出')


def stop_perception(include_legacy=True):
    roots=set(legacy('perception')) if include_legacy else set()
    table=processes()
    for path in state_directory().glob('*.lock'):
        try:
            record=json.loads(path.read_text());pid=record['pid'];p=table.get(pid)
            valid_role=record['role']=='camera' or (record['role'] in ('vision','perception') and record['domain']==os.environ.get('ROS_DOMAIN_ID','0'))
            managed=p and any(a.endswith('/scripts/managed_service.py') or a.endswith('/scripts/perception.py') for a in p['argv'])
            if valid_role and managed and p['start']==record['start']:roots.add(pid)
        except (OSError,ValueError,KeyError):continue
    remaining=terminate(roots)
    if remaining:raise RuntimeError('仍未退出的相机/视觉进程：'+str(remaining))
    print('相机和视觉进程已停止；未操作机器人驱动。',flush=True)


def stop_owned_perception():
    """Recover only this project's recorded supervisors in this ROS domain."""
    domain=os.environ.get('ROS_DOMAIN_ID','0')
    table=processes();roots=set()
    scripts=Path(__file__).resolve().parent
    for path in state_directory().glob('*.lock'):
        try:
            record=json.loads(path.read_text());pid=record['pid'];p=table.get(pid)
            if record['role'] not in ('camera','vision','perception') or record['domain']!=domain:continue
            expected=scripts/('perception.py' if record['role']=='perception' else 'managed_service.py')
            if p and p['start']==record['start'] and str(expected) in p['argv'] and same_domain(pid):roots.add(pid)
        except (OSError,ValueError,KeyError):continue
    remaining=terminate(roots)
    if remaining:raise RuntimeError('相机/视觉子进程尚未退出：'+str(remaining))
    # Unknown or cross-domain owners are deliberately left alone.
    conflicts=legacy('perception')
    if conflicts:raise RuntimeError('存在未归属当前服务的相机/视觉进程，未操作：'+str(conflicts))


if __name__=='__main__':
    parser=argparse.ArgumentParser()
    parser.add_argument('role',choices=['camera','vision','stop'])
    parser.add_argument('command',nargs=argparse.REMAINDER)
    args=parser.parse_args()
    if args.role=='stop':stop_perception()
    else:
        command=args.command[1:] if args.command[:1]==['--'] else args.command
        if not command:parser.error('Missing service command')
        raise SystemExit(supervise(args.role,[command]))
