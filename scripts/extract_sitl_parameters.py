#!/usr/bin/env python3
"""Extract parameter history and actuator samples from a SITL DataFlash BIN.

Use the local Python installation with pymavlink; originals are read-only.
"""
import argparse
from collections import Counter
import csv
import hashlib
import json
from pathlib import Path

from pymavlink import mavutil


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('bin')
    parser.add_argument('--output', required=True)
    args=parser.parse_args()
    source=Path(args.bin).resolve()
    directory=Path(args.output).resolve()
    directory.mkdir(parents=True, exist_ok=False)
    log=mavutil.mavlink_connection(str(source))
    parameters={}
    counts=Counter()
    with (directory/'parameter_history.jsonl').open('w') as history, (directory/'actuators.csv').open('w',newline='') as stream:
        writer=csv.writer(stream)
        writer.writerow(['time_us','channel1_pwm','channel2_pwm','channel3_pwm','channel4_pwm'])
        while True:
            msg=log.recv_match()
            if msg is None:
                break
            kind=msg.get_type()
            counts[kind]+=1
            if kind=='PARM':
                parameters[msg.Name]=msg.Value
                history.write(json.dumps(msg.to_dict())+'\n')
            elif kind=='RCOU':
                writer.writerow([msg.TimeUS]+[getattr(msg,'C%d'%i,None) for i in range(1,5)])
    log.close()
    report=dict(source=str(source), source_sha256=hashlib.sha256(source.read_bytes()).hexdigest(),
                extraction_sha256=hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
                parameters=parameters, message_counts=dict(counts), evidence='config_only for parameters; measured for actuator samples')
    (directory/'parameters.json').write_text(json.dumps(report,indent=2)+'\n')
    selected={name:value for name,value in parameters.items() if name in ('FRAME_CLASS','FRAME_TYPE','WP_SPD','WP_ACC',
              'SIM_FRM_MASS','SIM_FRM_REF_SPD','SIM_FRM_REF_ANG','SIM_WIND_SPD','SIM_WIND_TURB')}
    print(source, selected, 'RCOU samples:', counts['RCOU'])


if __name__=='__main__':
    main()
