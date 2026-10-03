#!/usr/bin/env python3
"""Summarize all planned repeats, preserving failures and missing attempts."""
import argparse
import csv
import json
import math
from pathlib import Path
import statistics

from uav_formation.experiment_artifacts import write_json
from uav_formation.paper_metrics import compute


def summarize(root):
    protocol=json.loads((root/'batch_protocol.json').read_text())
    rows=[]; detailed=[]
    for planned in protocol['runs']:
        directory=root/planned['id']
        row=dict(id=planned['id'],scene=planned['scene'],method=planned['method'],seed=planned['seed'],status='NOT_RUN')
        if (directory/'flight_status.json').exists():
            status=json.loads((directory/'flight_status.json').read_text()); row['status']=status['status']
            row['failure']=status.get('error','')
            try:
                m=compute(directory)
                row.update(completed=m['completed'],valid_telemetry=m.get('valid_telemetry',False),
                    duration_s=m.get('tracking_duration'),radius_m=m.get('selected_radius'),
                    mission_duration_s=status.get('duration'),minimum_interval_width_m_s=m.get('minimum_interval_width'),
                    shape_rmse_m=m.get('shape_rmse'),minimum_separation_m=m.get('minimum_separation'),
                    planning_seconds=m.get('planning_seconds'))
                if m['members']:
                    row['fleet_tracking_rmse_m']=math.sqrt(statistics.mean(v['tracking_rmse']**2 for v in m['members'].values()))
                    row['tracking_max_m']=max(v['tracking_max'] for v in m['members'].values())
                    row['max_member_tangential_violation_fraction']=max(v['violation_time_fraction']['tangential_lower']+v['violation_time_fraction']['tangential_upper'] for v in m['members'].values())
                    row['max_clock_drift_s']=max(v['max_clock_drift'] for v in m['members'].values())
                    configured=json.loads((directory/'config.json').read_text())['members']
                    platforms={str(v['member_id']):v['platform'] for v in configured}
                    for member_id,values in m['members'].items():
                        detailed.append(dict(run=planned['id'],scene=planned['scene'],method=planned['method'],seed=planned['seed'],
                                             member=member_id,platform=platforms[str(member_id)],**values))
            except Exception as exc:
                row['metric_error']=repr(exc)
        rows.append(row)
    fields=list(dict.fromkeys(key for row in rows for key in row))
    with (root/'all_runs.csv').open('w',newline='') as stream:
        writer=csv.DictWriter(stream,fieldnames=fields); writer.writeheader(); writer.writerows(rows)
    write_json(root/'all_member_metrics.json',detailed)
    constraint_rows=[]
    for member in detailed:
        for constraint,value in member['violation_time_fraction'].items():
            constraint_rows.append(dict(run=member['run'],scene=member['scene'],method=member['method'],
                seed=member['seed'],member=member['member'],constraint=constraint,
                violation_time_fraction=value,minimum_margin=member['minimum_margins'][constraint],
                margin_unit='rad' if constraint=='bank' else 'rad/s' if constraint=='yaw_rate' else
                'm/s^2' if constraint in ('lateral_accel','tangential_lower','tangential_upper') else 'm/s'))
    if constraint_rows:
        with (root/'all_constraints.csv').open('w',newline='') as stream:
            writer=csv.DictWriter(stream,fieldnames=list(constraint_rows[0]))
            writer.writeheader(); writer.writerows(constraint_rows)
    groups={}
    quantities=['fleet_tracking_rmse_m','tracking_max_m','shape_rmse_m','minimum_separation_m','duration_s',
                'radius_m','planning_seconds','max_member_tangential_violation_fraction','max_clock_drift_s',
                'mission_duration_s','minimum_interval_width_m_s']
    for row in rows:
        groups.setdefault(row['scene']+'_'+row['method'],[]).append(row)
    summary={}
    for name,group in groups.items():
        success=[r for r in group if r.get('completed') and r.get('valid_telemetry') and not r.get('metric_error')]
        result=dict(planned=len(group),attempted=sum(r['status']!='NOT_RUN' for r in group),
                    completed=len(success),failed=[r for r in group if r['status'] not in ('NOT_RUN','COMPLETED')],
                    missing=[r['id'] for r in group if r['status']=='NOT_RUN'],statistics={})
        for key in quantities:
            values=[r[key] for r in success if r.get(key) is not None]
            result['statistics'][key]=dict(n=len(values),mean=statistics.mean(values) if values else None,
                sample_sd=statistics.stdev(values) if len(values)>1 else None,min=min(values) if values else None,max=max(values) if values else None)
        summary[name]=result
    write_json(root/'summary.json',summary)
    lines=['# 正式实验结果汇总','',
        '所有计划运行均列入 all_runs.csv。均值和样本标准差仅汇总完成且遥测有效的运行；失败部分另列，不补零、不删除。',
        '重复单位为固定模型下的初始位置微扰，不代表不同机型或独立真实飞行。','',
        '| 组别 | 完成/计划 | 编队位置 RMS (m) | 形状 RMS (m) | 最小间距 (m) | 时间 (s) | 半径 (m) |',
        '|---|---:|---:|---:|---:|---:|---:|']
    def fmt(item):
        return '未获得' if item['mean'] is None else ('%.3f ± %.3f'%(item['mean'],item['sample_sd']) if item['sample_sd'] is not None else '%.3f'%item['mean'])
    for name,result in summary.items():
        stats=result['statistics']
        lines.append('| %s | %d/%d | %s | %s | %s | %s | %s |'%(name,result['completed'],result['planned'],
            *(fmt(stats[k]) for k in ('fleet_tracking_rmse_m','shape_rmse_m','minimum_separation_m','duration_s','radius_m'))))
    lines+=['','## 实际约束检查','',
        '以下每个数先在单次运行内取成员违反时间占比的最大值，再在完成且遥测有效的重复之间取均值和样本标准差。'
        '单位为百分比；各约束分开统计，不能相加解释为全队联合违反率。完整逐机数据见 all_constraints.csv。','',
        '| 组别 | 速度下界 (%) | 速度上界 (%) | 横向加速度 (%) | 切向加速度两侧 (%) | 滚转角 (%) | 偏航角速度 (%) |',
        '|---|---:|---:|---:|---:|---:|---:|']
    for name,group in groups.items():
        cells=[]
        for constraint in ('speed_lower','speed_upper','lateral_accel','tangential','bank','yaw_rate'):
            values=[]
            for run in group:
                if not run.get('completed') or not run.get('valid_telemetry') or run.get('metric_error'):
                    continue
                candidates=[]
                for member in detailed:
                    if member['run']!=run['id']:
                        continue
                    fractions=member['violation_time_fraction']
                    if constraint=='tangential':
                        candidates.append(fractions['tangential_lower']+fractions['tangential_upper'])
                    elif constraint in fractions:
                        candidates.append(fractions[constraint])
                if candidates:
                    values.append(100*max(candidates))
            cells.append('不适用/无数据' if not values else '%.3f'%statistics.mean(values)+
                         (' ± %.3f'%statistics.stdev(values) if len(values)>1 else ''))
        lines.append('| '+name+' | '+' | '.join(cells)+' |')
    lines+=['','## 实际观测范围','',
            '以下为完成且遥测有效运行中所有成员的观测极值，不是新的平台能力界。多旋翼速度为水平地速，固定翼为空速。','',
            '| 组别/平台 | 速度最小–最大 (m/s) | 最大绝对滚转 (deg) | 最大高度偏差 (m) | 最大仿真时钟偏差 (s) |',
            '|---|---:|---:|---:|---:|']
    accepted={r['id'] for r in rows if r.get('completed') and r.get('valid_telemetry') and not r.get('metric_error')}
    for name in groups:
        for platform in ('multirotor','fixedwing'):
            subset=[v for v in detailed if v['run'] in accepted and v['scene']+'_'+v['method']==name and v['platform']==platform]
            if subset:
                lines.append('| %s / %s | %.3f–%.3f | %.3f | %.3f | %.3f |'%(name,platform,
                    min(v['speed_min'] for v in subset),max(v['speed_max'] for v in subset),
                    max(v['bank_abs_max_deg'] for v in subset),max(v['altitude_error_max'] for v in subset),
                    max(v['max_clock_drift'] for v in subset)))
    lines+=['','任务完成不等于所有实际约束通过。切向加速度超限须在论文中明确报告；'
            '开发运行的 SIM2 真值独立复核也观察到越界，不能统一归因为遥测噪声。','',
            '## 未完成与失败','']
    unsuccessful=[r for r in rows if not r.get('completed') or not r.get('valid_telemetry') or r.get('metric_error')]
    lines+=['- '+r['id']+': '+r['status']+' '+str(r.get('failure') or r.get('metric_error') or '') for r in unsuccessful]
    if not unsuccessful:
        lines.append('本批次所有计划任务均完成且遥测有效。历史失败运行仍保留在 selection/calibration/development 与 formal_v1/v2。')
    lines+=['','## 结论边界','',
        '- F/A3 的可行性指离散参考约束通过独立稠密复核，不是实际飞控全时刻无越界的证明。',
        '- 各成员实际速度、滚转、切向/横向加速度和偏航角速度违反率见 all_member_metrics.json；不能只引用跟踪误差。',
        '- C 固定使用 320 m 和 14 m/s，与其他方法起止速度一致。大半径会改变路径长度，不能预设完整方法一定更快。',
        '- A0/A1/A2 及不可行的 A3 为离线受保护对照，没有对应未经干预的飞行曲线。',
        '- 固定翼为外部横向制导与 ArduPlane 滚转/TECS，未使用固定翼 NMPC。',
        '- 原始 BIN、tlog、配置和源文件快照保留在各运行的 raw_evidence.tar.gz；archive_manifest.json 给出逐文件和压缩包哈希。',
        '- 固定翼滚转角速度能力尚未标定；执行器 PWM/油门已记录，但未建立完整饱和包线。']
    (root/'RESULTS.md').write_text('\n'.join(lines)+'\n')
    return summary


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__); parser.add_argument('root')
    result=summarize(Path(parser.parse_args().root).resolve())
    print(json.dumps({k:dict(completed=v['completed'],planned=v['planned']) for k,v in result.items()},indent=2))
