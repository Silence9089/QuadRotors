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


def ShowData(t, x, y, z):
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
    plt.pause(0.001)


def SerialInit():
    """
    初始化串口
    :return:
    """
    ser = serial.Serial('COM7', 9600, timeout=0.01)
    if ser.isOpen():
        print('串口已打开')
    else:
        print('串口未打开')


#测试我的函数好不好使
if __name__ == '__main__':
    i = 0
    x = np.array([])
    y1 = np.array([])
    y2 = np.array([])
    y3 = np.array([])

    PltInit()

    while True:
        x = DataTrim(x, i)
        y1 = DataTrim(y1, math.sin(i))
        y2 = DataTrim(y2, math.cos(i))
        y3 = DataTrim(y3, math.tan(i))
        ShowData(x, y1, y2, y3)
        i += 0.1
