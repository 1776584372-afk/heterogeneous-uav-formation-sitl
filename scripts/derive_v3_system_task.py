#!/usr/bin/env python3
"""Derive a reduced-member v3 system precheck without changing its contract."""
import argparse
from copy import deepcopy
from pathlib import Path

import yaml

from uav_formation.experiment_config import load_config


def derive(source, platform, count=1):
    cfg, _ = load_config(source)
    available = [deepcopy(member) for member in cfg['members']
                 if member['platform'] == platform]
    if len(available) < count:
        raise ValueError('base task has insufficient ' + platform + ' members')
    selected = available[:count]
    slots = [0.] if count == 1 else [10. * (index - (count - 1) / 2)
                                     for index in range(count)]
    for index, (member, slot) in enumerate(zip(selected, slots)):
        member['member_id'] = index
        member['r_y'] = slot
    cfg['members'] = selected
    cfg['name'] = 'v3_system_precheck_%s_%d' % (platform, count)
    cfg['system_precheck'] = {
        'derived_from': str(Path(source).resolve()),
        'change': 'member count and IDs/slots only; frozen platform limits unchanged',
        'formal_result': False,
    }
    return cfg


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--source', type=Path, required=True)
    parser.add_argument('--platform', choices=['multirotor', 'fixedwing'], required=True)
    parser.add_argument('--count', type=int, default=1)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    if args.count < 1:
        parser.error('count must be positive')
    cfg = derive(args.source, args.platform, args.count)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    if args.output.exists():
        raise FileExistsError(args.output)
    args.output.write_text(yaml.safe_dump(cfg, sort_keys=False))
    load_config(args.output)
    print(args.output.resolve())


if __name__ == '__main__':
    main()
