"""Pure data capture/selection for right-arm approach trajectories; no ROS calls."""
import copy
import json
import math
from pathlib import Path


class ApproachCapture:
    def __init__(self):
        self.active=None

    def begin(self,event,now):
        if event.get('arm')!='right' or event.get('stage')!='APPROACH':return False
        self.active={'schema':1,'arm':'right','phase':'APPROACH','direction':event.get('direction'),
                     'start_monotonic':now,'event':copy.deepcopy(event),'samples':[],
                     'errors':[],'complete':False,'path_safety_verified':False}
        return True

    def error(self,reason):
        if self.active is not None and reason not in self.active['errors']:
            self.active['errors'].append(reason)

    def sample(self,now,right,left,waist,waist_age,state,fault=False):
        if self.active is None:return
        vectors=(right,left,waist)
        if any(len(v)!=n or not all(math.isfinite(x) for x in v) for v,n in zip(vectors,(7,7,3))):
            self.error('关节反馈缺失或非有限值');return
        if not math.isfinite(waist_age) or waist_age>.6:self.error('腰部反馈超过 0.6 秒未更新')
        if fault:self.error('记录期间驱动报告故障')
        t=now-self.active['start_monotonic'];samples=self.active['samples']
        if samples and t-samples[-1]['t_s']>.2:self.error('关节采样间隔超过 0.2 秒')
        if samples and t<=samples[-1]['t_s']:return
        samples.append({'t_s':t,'right_q_rad':list(right),'left_q_rad':list(left),
                        'waist_q_rad':list(waist),'waist_receive_age_s':waist_age,'driver_state':int(state)})

    def finish(self,reason,complete=False):
        if self.active is None:return None
        result=self.active;self.active=None
        result['end_reason']=reason;result['complete']=complete
        if len(result['samples'])<2:result['errors'].append('有效采样不足 2 帧')
        result['capture_valid']=complete and not result['errors']
        result['duration_s']=result['samples'][-1]['t_s'] if result['samples'] else 0.
        return result


def select_record(record,end_s=None):
    if not record.get('capture_valid'):raise ValueError('记录不完整或存在采样错误，不能选用')
    result=copy.deepcopy(record)
    if end_s is not None:
        if not math.isfinite(end_s) or not 0<end_s<=result['duration_s']:
            raise ValueError('截取终点必须在录制时长之内')
        result['samples']=[s for s in result['samples'] if s['t_s']<=end_s]
    if len(result['samples'])<2:raise ValueError('截取后采样不足')
    result['duration_s']=result['samples'][-1]['t_s']
    result['selected_for_review']=True
    result['path_safety_verified']=False
    return result


def save_json(path,value):
    path=Path(path);path.parent.mkdir(parents=True,exist_ok=True)
    temporary=path.with_suffix('.json.tmp')
    temporary.write_text(json.dumps(value,ensure_ascii=False,indent=2,allow_nan=False))
    temporary.replace(path)
