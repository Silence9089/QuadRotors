import numpy as np
import serial
import time
import matplotlib.pyplot as plt
import math


def PltInit():
    plt.ion()


def DataTrim(vector, new_data, length=100):
    """
    :param vector: 数据数组
    :param new_data: 新数据
    :param length: 要求的数据数组长度，默认100，若数据数组长度不足要求值，则自动补零
    :return: 加入新数据后的数据数组
    """
    vector = np.append(vector, new_data)
    if len(vector) < length:
        new_vector = np.zeros(length)
        new_vector[-len(vector):] = vector
    else:
        new_vector = np.delete(vector, 0)
    return new_vector


def ShowData(t, x, y, z, w):
    """
    显示数据，使用前需要先用DataTrim函数修剪数据
    :param t: 横坐标（时间）
    :param x:
    :param y:
    :param z:
    """
    plt.figure('QuadRotors')
    plt.clf()
    plt.subplot(221)
    plt.plot(t, x)
    plt.subplot(222)
    plt.plot(t, y)
    plt.subplot(223)
    plt.plot(t, z)
    plt.subplot(224)
    plt.plot(t, w)
    plt.pause(0.001)


#测试我的函数好不好使
if __name__ == '__main__':
    ser = serial.Serial('COM4', 9600, timeout=0.01)
    if ser.isOpen():
        print('串口已打开')
    else:
        print('串口未打开')
    t = 0

    PltInit()

    t = np.array([])
    x = np.array([])
    y = np.array([])
    z = np.array([])
    w = np.array([])
    t0 = 0

    while True:
        data = str(ser.readall())
        # print(data)
        index = data.find(r'\x00') + 4
        if len(data[index:-1]) < 19:
            continue
        t = DataTrim(t, t0+1)
        x = DataTrim(x, int(data[index:index + 4]))
        y = DataTrim(y, int(data[index + 4:index + 8]))
        z = DataTrim(z, int(data[index + 8:index + 12]))
        w = DataTrim(w, int(data[index + 12:index + 16]))
        ShowData(t, x, y, z, w)
        t0 += 1
