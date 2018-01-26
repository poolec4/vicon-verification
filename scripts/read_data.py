import argparse
import numpy as np
import matplotlib.pyplot as plt
import pdb
import sys
import time

from plot_funcs import *


def vee(S):
    s = np.array([-S[1, 2], S[0, 2], -S[0, 1]])
    return s.T


parser = argparse.ArgumentParser(
    description=(
        'Reads and plots the save file of FDCL flight software.' ))
parser.add_argument(
    '-f', '--file',
    default='../test0.txt',
    help='specify the file name, default FILE="../test0.txt"')
parser.add_argument(
    '-a', '--all',
    default=False, action='store_true',
    help='plot all data')
parser.add_argument(
    '-x', '--pos',
    default=False, action='store_true',
    help='plot position data')
parser.add_argument(
    '-v', '--velo',
    default=False, action='store_true',
    help='plot velocity data')
parser.add_argument(
    '-W', '--omega',
    default=False, action='store_true',
    help='plot angular velocity data')
parser.add_argument(
    '-R', '--rot',
    default=False, action='store_true',
    help='plot rotation matrix data')
parser.add_argument(
    '-e', '--error',
    default=False, action='store_true',
    help='plot all error data')
parser.add_argument(
    '-b', '--bias',
    default=False, action='store_true',
    help='plot estimated bias data')
parser.add_argument(
    '-m', '--moment',
    default=False, action='store_true',
    help='plot moments data')
if len(sys.argv)==1:
    parser.print_help()
    sys.exit(1)

args = parser.parse_args()

plot_all = args.all
plot_x = args.pos
plot_v = args.velo
plot_R = args.rot
plot_W = args.omega
plot_errors = args.error
plot_b = args.bias
plot_M = args.moment

t, tI, tV, x0, x1, x2, v0, v1, v2, a0, a1, a2, W0, W1, W2,\
    R00, R01, R02, R10, R11, R12, R20, R21, R22, \
    ba0, ba1, ba2, bW0, bW1, bW2, \
    WI0, WI1, WI2, aI0, aI1, aI2, \
    RI00, RI01, RI02, RI10, RI11, RI12, RI20, RI21, RI22, \
    xV0, xV1, xV2, \
    RV00, RV01, RV02, RV10, RV11, RV12, RV20, RV21, RV22, \
    Rd00, Rd01, Rd02, Rd10, Rd11, Rd12, Rd20, Rd21, Rd22, \
    Wd0, Wd1, Wd2, Wddot0, Wddot1, Wddot2,\
    eR0, eR1, eR2, eW0, eW1, eW2, eI0, eI1, eI2, \
    M0, M1, M2, f0, f1, f2, f3 = \
    np.loadtxt(args.file, delimiter=", ", usecols=range(88), unpack=True)

N = len(t)
R = np.zeros((3, 3, N))
W_diff = np.zeros((3, N - 1))

for i in range(0, N):
    R[0, 0, i] = R00[i]
    R[0, 1, i] = R01[i]
    R[0, 2, i] = R02[i]
    R[1, 0, i] = R10[i]
    R[1, 1, i] = R11[i]
    R[1, 2, i] = R12[i]
    R[2, 0, i] = R20[i]
    R[2, 1, i] = R21[i]
    R[2, 2, i] = R22[i]

RI = np.zeros((3, 3, N))
for i in range(0, N):
    RI[0, 0, i] = RI00[i]
    RI[0, 1, i] = RI01[i]
    RI[0, 2, i] = RI02[i]
    RI[1, 0, i] = RI10[i]
    RI[1, 1, i] = RI11[i]
    RI[1, 2, i] = RI12[i]
    RI[2, 0, i] = RI20[i]
    RI[2, 1, i] = RI21[i]
    RI[2, 2, i] = RI22[i]
I_i0 = np.argmin(abs(tI - t.min()))

RV = np.zeros((3, 3, N))
for i in range(0, N):
    RV[0, 0, i] = RV00[i]
    RV[0, 1, i] = RV01[i]
    RV[0, 2, i] = RV02[i]
    RV[1, 0, i] = RV10[i]
    RV[1, 1, i] = RV11[i]
    RV[1, 2, i] = RV12[i]
    RV[2, 0, i] = RV20[i]
    RV[2, 1, i] = RV21[i]
    RV[2, 2, i] = RV22[i]
V_i0 = np.argmin(abs(tV - t.min()))

for i in range(0, N - 1):
    h = t[i + 1] - t[i]
    W_diff[:, i] = vee(R[:, :, i].T.dot(R[:, :, i + 1] - R[:, :, i])) / h

if plot_x or plot_all:
    plot_31_2(t, [x0, x1, x2],
          tV[V_i0:], [xV0[V_i0:], xV1[V_i0:], xV2[V_i0:]],
          title='x',
          legend=['Est.', 'Vicon'])

if plot_v or plot_all:
    plot_31_2(t, [v0, v1, v2],
          t[0:-1], [np.diff(x0) / np.diff(t),
          np.diff(x1) / np.diff(t), np.diff(x2) / np.diff(t)],
          title='v', legend=['Est.', 'Diff. position'])

if plot_b or plot_all:
    plot_31_1(t, [ba0, ba1, ba2], title='b_a')
    plot_31_1(t, [bW0, bW1, bW2], title='b_\Omega')

if plot_R or plot_all:
    plt.figure()
    for i in range(0, 3):
        for j in range(0, 3):
            k = 3 * i + j + 1
            axes = plt.subplot(3, 3, k)
            plt.plot(t, R[i, j, :], 'b', label='Est.')
            plt.plot(tI[I_i0:], RI[i, j, I_i0:], 'r', label='IMU')
            plt.plot(tV[V_i0:], RV[i, j, V_i0:], 'k', label='Vicon')
            axes.set_ylim([-1, 1])
    plt.suptitle(r'$R$')
    axes.legend()

if plot_W or plot_all:
    plt.figure(5)
    plt.subplot(311)
    plt.plot(t[0:-1], W_diff[0, :], 'g')
    plt.plot(tI[I_i0:], WI0[I_i0:], 'r')
    plt.plot(t, W0, 'b')
    plt.subplot(312)
    plt.plot(t[0:-1], W_diff[1, :], 'g')
    plt.plot(tI[I_i0:], WI1[I_i0:], 'r')
    plt.plot(t, W1, 'b')
    plt.subplot(313)
    plt.plot(t[0:-1], W_diff[2, :], 'g')
    plt.plot(tI[I_i0:], WI2[I_i0:], 'r')
    plt.plot(t, W2, 'b')

    plt.suptitle(r'$\Omega$')


if plot_errors or plot_all:
    plot_31_1(t, [eR0, eR1, eR2], title='e_R')
    plot_31_1(t, [eW0, eW1, eW2], title='e_\Omega')
    plot_31_1(t, [eI0, eI1, eI2], title='e_I')

if plot_M or plot_all:
    plot_31_1(t, [M0, M1, M2], title='M')


try:
    print('Press Ctrl+C to close all figures.')
    plt.show()
except KeyboardInterrupt:
    plt.close('all')
