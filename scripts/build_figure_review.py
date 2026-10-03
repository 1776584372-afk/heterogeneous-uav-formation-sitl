#!/usr/bin/env python3
"""Create labelled QA contact sheets; original scientific exports stay untouched."""
import argparse
import math
from pathlib import Path

from PIL import Image,ImageDraw,ImageFont


def build(source,destination):
    paths=sorted(source.glob('*.png'))
    if not paths:
        return
    width,height=900,490
    sheet=Image.new('RGB',(2*width,math.ceil(len(paths)/2)*height),'white')
    draw=ImageDraw.Draw(sheet)
    font=ImageFont.truetype('/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf',18)
    for i,path in enumerate(paths):
        x=(i%2)*width; y=(i//2)*height
        with Image.open(path) as original:
            view=original.convert('RGB'); view.thumbnail((width,height-35))
            sheet.paste(view,(x,y+35))
        label=source.parent.name if source.name=='trace_plots' else source.name
        draw.text((x+12,y+8),label+' / '+path.stem,fill='black',font=font)
    destination.parent.mkdir(parents=True,exist_ok=True)
    sheet.save(destination)


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('source',type=Path); parser.add_argument('destination',type=Path)
    args=parser.parse_args(); build(args.source,args.destination)
