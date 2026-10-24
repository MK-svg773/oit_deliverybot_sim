#!/usr/bin/env python3
"""Plot recorded trials, including unsuccessful and infrastructure outcomes."""
import argparse
import csv
from collections import defaultdict
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import numpy as np


def plot_batch(root):
    root=Path(root)
    with open(root/'results.csv') as stream:results=list(csv.DictReader(stream))
    if not results:return
    fig,axes=plt.subplots(2,2,figsize=(13,9),constrained_layout=True)
    groups=defaultdict(list)
    for row in results:
        label=row['scenario']+'/'+row['method']
        groups[label].append(row)
        path=Path(row['output'])/'samples.csv'
        if not path.is_file():continue
        with open(path) as stream:samples=list(csv.DictReader(stream))
        if not samples:continue
        def values(key):return np.array([float(s[key]) if s[key] else np.nan for s in samples])
        desc=f"{label} #{row['trial']} ({row['outcome']})"
        axes[0,0].plot(values('x_m'),values('y_m'),label=desc)
        axes[0,1].plot(values('time_s'),values('speed_mps'),label=desc)
        axes[1,0].plot(values('time_s'),values('clearance_m'),label=desc)
    axes[0,0].set(xlabel='world x [m]',ylabel='world y [m]',title='True trajectories')
    axes[0,0].set_aspect('equal',adjustable='datalim')
    axes[0,1].set(xlabel='simulation time [s]',ylabel='speed [m/s]',title='Measured speed')
    axes[1,0].set(xlabel='simulation time [s]',ylabel='footprint gap [m]',title='Gap to controlled obstacle')
    labels=list(groups)
    success=[sum(r['outcome']=='success' for r in groups[k])/len(groups[k]) for k in labels]
    bars=axes[1,1].bar(range(len(labels)),success)
    axes[1,1].set(ylim=(0,1.25),title='Success / all attempts (including infrastructure errors)',ylabel='fraction')
    axes[1,1].set_xticks(range(len(labels)),labels,rotation=35,ha='right')
    for bar,label in zip(bars,labels):
        rows=groups[label];fail=defaultdict(int)
        for r in rows:fail[r['outcome']]+=1
        desc='\n'.join(f'{k}: {v}' for k,v in fail.items())
        axes[1,1].text(bar.get_x()+bar.get_width()/2,bar.get_height()+.02,desc,ha='center',fontsize=7)
    for ax in axes.flat:ax.grid(alpha=.25)
    if axes[0,0].lines:axes[0,0].legend(fontsize=6,loc='best')
    fig.savefig(root/'comparison.png',dpi=160)
    plt.close(fig)


if __name__=='__main__':
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('batch',type=Path)
    plot_batch(parser.parse_args().batch)
