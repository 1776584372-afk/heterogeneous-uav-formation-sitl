#!/usr/bin/env python3
"""Index historical attempts, including failed calibration steps within completed sweeps."""
import argparse
import json
from pathlib import Path

from uav_formation.experiment_artifacts import write_json


def index(root):
    entries=[]
    for group in ('selection','calibration','development'):
        for directory in sorted((root/group).iterdir()):
            if not directory.is_dir():
                continue
            for filename in ('result.json','flight_status.json','precheck_result.json'):
                path=directory/filename
                if not path.exists():
                    continue
                value=json.loads(path.read_text())
                if not isinstance(value,dict):
                    continue
                failed=[s for s in value.get('steps',[]) if s.get('status')=='FAIL']
                entries.append(dict(directory=str(directory.relative_to(root)),source=filename,
                    reported_status=value.get('status','UNSPECIFIED'),error=value.get('error'),
                    failed_steps=failed,passed_steps=sum(s.get('status')=='PASS' for s in value.get('steps',[])),
                    total_steps=len(value.get('steps',[]))))
    write_json(root/'historical_attempts.json',entries)
    lines=['# 历史尝试索引','',
           '保留原报告状态；COMPLETED 可能仅表示阶梯测试程序结束，内部仍可能存在 FAIL 档位。'
           '地面接口测试、单机标定、多机开发运行不能混算为正式重复。','',
           '| 目录 | 报告文件 | 原状态 | 通过档位/总档位 | 失败档位数 |',
           '|---|---|---|---:|---:|']
    for row in entries:
        lines.append('| %s | %s | %s | %d/%d | %d |'%(row['directory'],row['source'],row['reported_status'],
                     row['passed_steps'],row['total_steps'],len(row['failed_steps'])))
    lines+=['','正式早期批次 formal_v1 的控制超时/中断与 formal_v2 的终点预测修复前运行另见各自批次状态。'
            'formal_v2 第五次飞行虽完成，但外层返回码因进程中断不可得，未填造返回码；不混入 formal_v3。',
            '全部错误详情、失败档位和判据保留在 historical_attempts.json 及对应原始目录。']
    (root/'HISTORICAL_ATTEMPTS.md').write_text('\n'.join(lines)+'\n')
    print('Indexed',len(entries),'original reports')


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__); parser.add_argument('root',type=Path)
    index(parser.parse_args().root)
