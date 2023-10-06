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
def calc_sensor_zeros(dll, sensor_port, limit, status):
    print('calclating zeros')
    # 連続読み込みモード開始
    if dll.SetSerialMode(sensor_port, True) == False:
            print('連続読み込みモードを開始できません')
            exit()

    force_data = (ctypes.c_double * 6)()  # 測定した力
    zeros = [0, 0, 0, 0, 0, 0]  # 0Nになる出力値
    repeat_time = 1000  # 繰り返し回数

    # 力を繰り返し取得
    i = 0
    while i < repeat_time:
        if dll.GetSerialData(sensor_port, force_data, ctypes.pointer(status)) == True:
            for j in range(len(limit)):
                zeros[j] += force_data[j] / 10000.0 * limit[j]
            i += 1

    # 力の平均を計算
    for j in range(len(limit)):
        zeros[j] = zeros[j] / repeat_time

    # 連続読み込みモードを停止
    if dll.SetSerialMode(sensor_port, False) == False:
        print('連続読み込みモードを停止できません')
        exit()

    return zeros


def main():
    # センサ関連の変数宣言
    sensor_port = 5  # センサのポート番号
    status = ctypes.c_char()  # センサ情報
    serial_num = (ctypes.c_char * 9)()  # シリアルナンバー
    limit =(ctypes.c_double * 6)()  # センサ定格
    data = (ctypes.c_double * 6)()  # センサ出力
    data_newton = (ctypes.c_double * 6)()  # 力[N]
    zeros = (ctypes.c_double * 6)  # 0Nがセンサに印加されたときの力[N]の値
    data_name = ['Fx', 'Fy', 'Fz', 'Mx', 'My', 'Mz']  # dataに格納されている値

    # モータ関連の変数宣言
    motor_port = 'COM8'  # モータコントローラが接続されるCOMポート
    bordrate = 9600  # シリアル通信のボーレート

    # dll読み込み
    sensor_dll_path = r'C:\Users\Shota\work\graduation_thesis_B4\pc\CfsUsb.dll'
    sensor_dll = ctypes.cdll.LoadLibrary(sensor_dll_path)
    sensor_dll.Initialize()  # dll初期化

    try:
        # ポートを開く
        if sensor_dll.PortOpen(sensor_port) == False:
            print('ポートが開けません')
            exit()
        # センサ定格を確認
        elif sensor_dll.GetSensorLimit(sensor_port, limit) == False:
            print('センサー定格が確認できません')
            exit()
        # シリアルナンバー確認
        elif sensor_dll.GetSensorInfo(sensor_port, serial_num) == False:
            print('シリアルナンバーが確認できません')
            exit()
        
        # 操作設定
        target_posion = float(input('input target position [mL]:'))
        target_time = float(input('input target time [s]:'))
        speed = int(input('input speed (1 ~ 255, max = 0)'))

        # データ保存用の変数宣言
        time_list = []  # 経過時間
        Fz_list = []  # 力測定値
        Fz_filtered_list = []  # フィルタ後の力
        target_position_list = []  # 位置目標値
        now_position = 0  # 現在の位置の目標値

        # センサキャリブレーション
        sensor_zeros = calc_sensor_zeros(sensor_dll, sensor_port, limit, status)

        send_data = [0, 1, 0]  # 送信用データ
        motor_serial = serial.Serial(motor_port, bordrate, timeout = 0.1)
        time.sleep(5)

        # sキーを押して動作開始
        print('press "s" key to start, "q" key to stop')
        while True:
            if keyboard.is_pressed('s'):
                break
        
        send_data = [0, 200, 0, 1]
        motor_serial.write(send_data)
        time.sleep(10)


        start_time = time.perf_counter()  # 開始時間
        time_passed = 0  # 経過時間

        while True:
            if sensor_dll.GetLatestData(sensor_port, data, ctypes.pointer(status)) == True: 
                time_passed = time.perf_counter() - start_time
                # センサデータ取得
                for i in range(len(limit)):  
                    data_newton[i] = limit[i] / 10000.0 * data[i] - sensor_zeros[i]
                
                # データをリストに格納
                time_list.append(time_passed)
                Fz = data_newton[data_name.index('Fz')]
                Fz_list.append(Fz)

                # モータコントローラへ送信
                send_data = [0, 0, 0, 0]
                motor_serial.write(send_data)
                time.sleep(0.2)

                # モータコントローラから受信
                read_data_binary = motor_serial.read_all()
                read_data = struct.unpack('3B', read_data_binary)
                now_position = read_data[1] + read_data[2] * 256
                print(now_position)

                # インターバル
                time.sleep(0.005) 

                if time_passed >= target_time:
                    send_data = [0, 0, 0]
            
            if keyboard.is_pressed('q'):  # qを押して停止
                send_data = [2, 0, 0, 0, 0]
                motor_serial.write(send_data)
                time.sleep(0.1)
                break

        # グラフ描画
        Fz_filtered_list = np.insert(Fz_filtered_list, 0, 0)
        plt.figure()   
        plt.plot(time_list, Fz_list)
        plt.ylim(0, 25)
        plt.xlabel('Time [s]')
        plt.ylabel('Force [N]')
        plt.grid()
        plt.savefig('result_fig\\result.png')
        plt.pause(5)

        # 結果csv保存
        speed_result_list = []
        for i in range(len(time_list) - 1):
            speed_result_list.append(- (target_position_list[i + 1] - target_position_list[i]) / (time_list[i + 1] - time_list[i]) / 13.235)
        #speed_result_list = signal.lfilter(b, a, speed_result_list)
        result = pd.DataFrame(zip(time_list, Fz_list, Fz_filtered_list, target_position_list, speed_result_list),
                               columns = ['Time [s]', 'Fz [N]', 'Fz filtered [N]','target position [-]', 'speed'])
        file_name = input('input file name: ')
        result.to_csv('result\\' + file_name + '.csv')   
        print('saving as csv completed')
        
    finally:  # 終了時の処理
        send_data = [2, 0, 0, 0, 0]
        motor_serial.write(send_data)
        time.sleep(0.01)
        motor_serial.close()
        sensor_dll.PortClose()
        sensor_dll.Finalize()
        del sensor_dll
        print('完了')

        
if __name__ == '__main__':
    main()