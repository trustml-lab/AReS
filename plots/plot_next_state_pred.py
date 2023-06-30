import os, sys
import numpy as np
import pickle
import argparse

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.backends.backend_pdf import PdfPages


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='plot')
    
    # parameters
    parser.add_argument('--data_fn', type=str, default='output/ares/data.pk')
    parser.add_argument('--baseline_data_fn', type=str, default='output/baseline/data.pk')

    parser.add_argument('--fig_root', type=str, default='output/ares_mav_test/figs')
    parser.add_argument('--fontsize', type=int, default=20)
    args = parser.parse_args()

    # init
    os.makedirs(args.fig_root, exist_ok=True)
    data = pickle.load(open(args.data_fn, 'rb'))
    data_baseline = pickle.load(open(args.baseline_data_fn, 'rb'))

    pv_next_pred = [d['y_pred'][1] for d in data]
    pv_next = [d['y'][1] for d in data]
    pv_next_pred_baseline = [d['y_pred'][1] for d in data_baseline]

    fig_fn = os.path.join(args.fig_root, f'plot_next_state_pred')
    print(fig_fn)
    with PdfPages(fig_fn + '.pdf') as pdf:
        plt.figure(1)
        plt.clf()

        h1 = plt.plot(pv_next, 'k', marker='s', markevery=10, label='Next velocity')
        h2 = plt.plot(pv_next_pred, 'g', label='Predeicted next velocity (ours)')
        h3 = plt.plot(pv_next_pred_baseline, 'r', label='Predeicted next velocity (baseline)')

        plt.xlabel('time step', fontsize=0.75*args.fontsize)
        plt.ylabel('velocity', fontsize=0.75*args.fontsize)
        plt.grid('on')
        plt.legend(handles=[h1[0], h2[0], h3[0]], fontsize=0.75*args.fontsize)
        plt.savefig(fig_fn+'.png', bbox_inches='tight')
        pdf.savefig(bbox_inches='tight')
