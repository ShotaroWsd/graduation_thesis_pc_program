import ctypes
import serial
import matplotlib.pyplot as plt
import time
import keyboard
import struct
import pandas as pd
import numpy as np
from scipy import signal

# センサの0点を測定，計算する関数
def calc_sensor_zeros(dll, port_num, limit, status):
    print('calclating zeros')
    # 連続読み込みモード開始
    if dll.SetSerialMode(port_num, True) == False:
            print('連続読み込みモードを開始できません')
            exit()

    force_data = (ctypes.c_double * 6)()  # 測定した力
    zeros = [0, 0, 0, 0, 0, 0]  # 0Nになる出力値
    repeat_time = 1000  # 繰り返し回数

    # 力を繰り返し取得
    i = 0
    while i < repeat_time:
        if dll.GetSerialData(port_num, force_data, ctypes.pointer(status)) == True:
            for j in range(len(limit)):
                zeros[j] += force_data[j] / 10000.0 * limit[j]
            i += 1

    # 力の平均を計算
    for j in range(len(limit)):
        zeros[j] = zeros[j] / repeat_time

    # 連続読み込みモードを停止
    if dll.SetSerialMode(port_num, False) == False:
        print('連続読み込みモードを停止できません')
        exit()

    return zeros


def main():
    # センサ関連の変数宣言
    sensor_port = 3  # センサのポート番号
    status = ctypes.c_char()  # センサ情報
    serial_num = (ctypes.c_char * 9)()  # シリアルナンバー
    limit =(ctypes.c_double * 6)()  # センサ定格
    data = (ctypes.c_double * 6)()  # センサ出力
    data_newton = (ctypes.c_double * 6)()  # 力[N]
    zeros = (ctypes.c_double * 6)  # 0Nがセンサに印加されたときの力[N]の値
    data_name = ['Fx', 'Fy', 'Fz', 'Mx', 'My', 'Mz']  # dataに格納されている値

    # モータ関連の変数宣言
    motor_port = 'COM7'  # モータコントローラが接続されるCOMポート
    bordrate = 9600  # シリアル通信のボーレート

    # dll読み込み
    sensor_dll_path = r'C:\Users\Shota\work\graduation_thesis_B4\pc\CfsUsb.dll'
    sensor_dll = ctypes.cdll.LoadLibrary(sensor_dll_path)
    sensor_dll.Initialize()  # dll初期化


if __name__ == '__main__':
    main()