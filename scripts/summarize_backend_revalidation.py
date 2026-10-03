#!/usr/bin/env python3
"""Index independent calibration and development flights without granting acceptance."""
import argparse
import hashlib
import json
from pathlib import Path

from scripts.analyze_backend_dynamics import analyze


def digest(path):
    h=hashlib.sha256()
    with path.open('rb') as stream:
        for chunk in iter(lambda:stream.read(1024*1024),b''):h.update(chunk)
    return h.hexdigest()


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('root');parser.add_argument('--output',required=True)
    args=parser.parse_args();root=Path(args.root).resolve();out=Path(args.output).resolve()
    out.mkdir(parents=True,exist_ok=False)
    summary=dict(calibrations=[],flights=[],planning_rejections=[],formal_batches_started=False,
                 continuous_capability_envelope_certified=False,raw_sha256={})
    parameter_source=root/'calibration_batch_01/plane_speed/events.jsonl'
    selected={'AIRSPEED_MIN','AIRSPEED_MAX','ROLL_LIMIT_DEG','TECS_TIME_CONST','TECS_THR_DAMP',
              'TECS_SPDWEIGHT','TECS_PTCH_DAMP','SIM_ARSPD_RND','SIM_WIND_SPD','SIM_WIND_TURB'}
    values={}
    with parameter_source.open() as stream:
        for line in stream:
            message=json.loads(line)['message']
            if message.get('mavpackettype')=='PARAM_VALUE' and message['param_id'] in selected:
                values[message['param_id']]=message['param_value']
    summary['plane_firmware_parameters']=dict(source=str(parameter_source),evidence='configuration_only',values=values)
    for path in sorted(root.rglob('result.json')):
        directory=path.parent
        if not (directory/'protocol.json').exists():continue
        result=json.loads(path.read_text())
        dynamics=analyze(directory)
        name=str(directory.relative_to(root))
        summary['calibrations'].append(dict(name=name,result=result,dynamics=dynamics))
    for path in sorted(root.rglob('flight_status.json')):
        directory=path.parent
        if json.loads(path.read_text()).get('status')=='REFERENCE_REJECTED':
            continue
        def read(name):
            file=directory/name
            return json.loads(file.read_text()) if file.exists() else None
        summary['flights'].append(dict(name=str(directory.relative_to(root)),
            status=read('flight_status.json'),acceptance=read('acceptance.json'),
            feedback=read('feedback_audit.json'),metrics=read('metrics.json'),config=read('config.json')))
    for path in sorted(root.rglob('candidates.json')):
        status=path.parent/'flight_status.json'
        if not status.exists() or json.loads(status.read_text()).get('status')=='REFERENCE_REJECTED':
            summary['planning_rejections'].append(dict(name=str(path.parent.relative_to(root)),
                candidates=json.loads(path.read_text())))
    names={'telemetry.csv','raw.tlog','result.json','flight.csv','feedback_solves.jsonl',
           'fixedwing_commands.jsonl','config.json','protocol.json','manifest.json',
           'defaults.parm','events.jsonl'}
    for path in sorted(root.rglob('*')):
        if path.is_file() and out not in path.parents and (path.name in names or path.suffix=='.BIN'):
            summary['raw_sha256'][str(path.relative_to(root))]=digest(path)
    frozen=Path('runs/paper_hetero/frozen_v1/capabilities_verified.yaml')
    baseline=Path('runs/final_validation_20260925/s3_smooth_01/config_snapshot/capabilities_verified.yaml')
    summary['frozen_capability_unchanged']=digest(frozen)==digest(baseline)
    summary['frozen_capability_sha256']=digest(frozen)
    (out/'summary.json').write_text(json.dumps(summary,indent=2,ensure_ascii=False,allow_nan=False)+'\n')
    lines=['---','date: 2026-09-26','experiment: direct_mavlink_backend_revalidation',
           'formal_qualification: false','---','', '# 当前后端标定与执行层复测','',
           '本目录只记录独立标定和开发复测。正式 A0/A1/A2/P、S1/S2/S3 重复批次尚未启动。',
           '跟踪通过、实际约束通过和完整任务完成分别判定；未完成任务的局部约束通过不构成验收。','',
           '## 条件与边界','',
           '- ArduCopter `x:freestyle.json` 与 ArduPlane `plane`，通用 SITL 模型，不是商品机型或实机。',
           '- 无风、相对高度 60 m；多旋翼直接 MAVLink PPVA，固定翼原生 course/airspeed/altitude 与 TECS。',
           '- 研究冻结限值未修改。多旋翼速度 0–20 m/s、切向加速度 ±0.4 m/s²、航向角速度 ±0.25 rad/s；固定翼空速 12–24 m/s、切向加速度 ±0.2 m/s²、滚转角 ±27°。这些数值不能统称已测得的真实极限。',
           '- 固定翼飞控实读配置：AIRSPEED_MIN=%g m/s、AIRSPEED_MAX=%g m/s、ROLL_LIMIT_DEG=%g°，与研究运行窗口不同。以上是配置证据，不能代替真实能力标定。'%(values['AIRSPEED_MIN'],values['AIRSPEED_MAX'],values['ROLL_LIMIT_DEG']),
           '- 无风不等于无传感器噪声；原有 SIM_ARSPD_RND=%g 保留，未关闭噪声以获取通过结果。'%values['SIM_ARSPD_RND'],
           '- 配置直线共同区间仍为 [12,20] m/s，仅在无风假设下将固定翼空速与多旋翼地速对应。离散测试不能证明整个区间连续可行。',
           '- 航向水平坐标速度与三维机体系速度分别保存；倾斜飞行时不得混称。滚转角速度记录响应范围，最大能力界仍未知。','',
           '## 标定证据','',
           '| 试验 | 通过的稳态跟踪点/已测点 | 退出完成 | 证据 |',
           '|---|---:|---|---|']
    for entry in summary['calibrations']:
        r=entry['result'];steps=r.get('steps',[])
        passed=sum(s['status']=='PASS' for s in steps)
        lines.append('| %s | %d/%d | %s | [%s](%s) |'%(entry['name'],passed,len(steps),
            r.get('exit_completed',False),'原始结果',root/entry['name']/'result.json'))
    lines+=['','“通过”采用事先冻结的稳态跟踪误差协议，不表示逐时刻动力学约束通过或包线认证。每个稳态窗口和整个变速/转弯阶段的未平滑导数均保存在 summary.json。','',
            '### 四向速度的坐标区别','',
            '| 方向 | 航向水平 20 m/s 指令时实测 (m/s) | 同时的三维机体系分量 (m/s) |',
            '|---|---:|---:|']
    for entry in summary['calibrations']:
        direction=entry['result'].get('protocol',{}).get('direction')
        if entry['dynamics']['platform']!='copter' or not direction:continue
        candidates=[s for s in entry['dynamics']['steady_windows'] if s['speed']==20. and 'metrics' in s]
        if not candidates:continue
        metrics=candidates[-1]['metrics'];axis='forward' if direction in ('forward','backward') else 'left'
        sign=-1 if direction in ('backward','right') else 1
        ranges=[]
        for key in ('heading_'+axis,'body_'+axis):
            limits=sorted(sign*metrics[key][part] for part in ('minimum','maximum'))
            ranges.append('%.3f–%.3f'%tuple(limits))
        lines.append('| %s | %s | %s |'%(direction,*ranges))
    lines+=['','方向分量按对应运动方向取正。这些是特定测试窗口内的观测值，不是已确认的三维机体系最大速度。','',
            '## 执行复测','',
            '| 试验 | 全航段完成 | 位置通过 | 已观测实际约束通过 | RMSE / 最大误差 (m) |',
            '|---|---|---|---|---|']
    for entry in summary['flights']:
        a=entry['acceptance'] or {};members=a.get('members',{})
        errors='; '.join('%s: %.3f / %.3f'%(i,m['tracking_rmse'],m['tracking_max']) for i,m in members.items())
        lines.append('| [%s](%s) | %s | %s | %s | %s |'%(entry['name'],root/entry['name'],
            a.get('full_route_completed','未知'),a.get('tracking_qualified','未知'),
            a.get('measured_constraints_passed','未知'),errors))
    lines+=['','所有新增执行策略均须显式开启开发开关，不能凭本表宣称正式验收。短控制周期、首段保持优化和连续命令积分试验的失败数据也全部保留。','',
            '## 几何拒绝','',
            *['- `%s`: `%s`'%(entry['name'],json.dumps(entry['candidates'],ensure_ascii=False)) for entry in summary['planning_rejections']],
            '',
            '## 复核入口','',
            '- `summary.json` 包含每个工况的原始数据索引、完整/稳态动态统计、飞行验收、配置及原始文件 SHA-256。',
            '- `s1_chain`、`s2_chain` 给出上一版参考、控制器/发送输出和实际响应的逐时刻对齐。',
            '- 本轮不覆盖原始实验，不回写能力界，不滤平峰值，不用优化器成功替代飞机动态验收。','']
    (out/'README.md').write_text('\n'.join(lines))
    print(out/'README.md')


if __name__=='__main__':main()
