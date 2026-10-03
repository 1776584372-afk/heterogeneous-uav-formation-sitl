#!/usr/bin/env python3
"""Audit every exported PDF without modifying scientific data or artwork."""
import argparse
import hashlib
import importlib.util
import json
from pathlib import Path
import sys

from uav_formation.experiment_artifacts import write_json


DEFAULT_QA=Path.home()/'.codex/skills/nature-figure/scripts'


def load_module(name,path):
    spec=importlib.util.spec_from_file_location(name,path)
    module=importlib.util.module_from_spec(spec)
    sys.modules[name]=module
    spec.loader.exec_module(module)
    return module


def audit(directory,qa_scripts=DEFAULT_QA):
    text_audit=load_module('paper_text_audit',qa_scripts/'audit_pdf_text.py')
    collision=load_module('paper_collision_audit',qa_scripts/'audit_figure_collisions.py')
    reports=[]
    for pdf in sorted(Path(directory).glob('*.pdf')):
        raw=pdf.read_bytes()
        glyphs=text_audit.audit_pdf(raw,5.)
        geometry=collision.audit_pdf(pdf)
        write_json(pdf.with_suffix('.text-audit.json'),glyphs)
        write_json(pdf.with_suffix('.collision-audit.json'),geometry)
        reports.append(dict(file=pdf.name,sha256=hashlib.sha256(raw).hexdigest(),
            glyph_pass=glyphs['auditable'] and glyphs['below_minimum_count']==0,
            collision_verdict=geometry['verdict'],collision_exit_code=collision.exit_code(geometry)))
    result=dict(figures=reports,automated_pass=bool(reports) and all(
        r['glyph_pass'] and r['collision_verdict']=='PASS' for r in reports),
        visual_review='Automated geometry checks do not replace visual/scientific inspection.')
    write_json(Path(directory)/'qa_summary.json',result)
    return result


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('directory'); parser.add_argument('--qa-scripts',type=Path,default=DEFAULT_QA)
    args=parser.parse_args(); result=audit(Path(args.directory),args.qa_scripts)
    print(json.dumps(result,indent=2))
    raise SystemExit(0 if result['automated_pass'] else 1)
