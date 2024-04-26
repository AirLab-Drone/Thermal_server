#!/usr/bin/env python3
import numpy as np 
import struct

class FirFrame:
    def __init__(self):
        self.FileFlag = bytes([0] * 4)
        self.OptiTrans = 0.0
        self.Emiss = 0.0
        self.Distance = 0.0
        self.AmbientTemperature = 0.0
        self.RelativeHumidity = 0.0
        self.Width = 0
        self.Height = 0
        self.Precision = bytes([0])
        self.IRData = None

def getTemperatureMartix(self) -> np.array:
        
        picture_frame = FirFrame()
        try:
            # 解析文件標記
            picture_frame.FileFlag = self.heat_map.read(4)

            # 獲取光學透過率
            picture_frame.OptiTrans = struct.unpack("f", self.heat_map.read(4))[0]

            # 獲取辐射率
            picture_frame.Emiss = struct.unpack("f", self.heat_map.read(4))[0]

            # 獲取拍攝距離
            picture_frame.Distance = struct.unpack("f", self.heat_map.read(4))[0]
            

            # 獲取環境溫度
            picture_frame.AmbientTemperature = struct.unpack("f", self.heat_map.read(4))[0]

            # 獲取相對濕度
            picture_frame.RelativeHumidity = struct.unpack("f", self.heat_map.read(4))[0]

            # 獲取圖像文件高度和寬度
            picture_frame.Height = struct.unpack("H", self.heat_map.read(2))[0]
            picture_frame.Width = struct.unpack("H", self.heat_map.read(2))[0]

            # 獲取精度
            picture_frame.Precision = self.heat_map.read(1)

            # 移動文件指針到 IRData 的開始位置
            self.heat_map.seek(64)

            # 讀取溫度矩陣
            matrix_size = picture_frame.Width * picture_frame.Height
            picture_frame.IRData = struct.unpack('h' * matrix_size, self.heat_map.read(2 * matrix_size))
            # 把IRData轉成np.array
            picture_frame.IRData = np.array(picture_frame.IRData).reshape(picture_frame.Height, picture_frame.Width)
            picture_frame.IRData = picture_frame.IRData / 10.0
            
            print(f'picture_frame.OptiTrans: {picture_frame.OptiTrans}')
            print(f'picture_frame.Emiss: {picture_frame.Emiss}')
            print(f'picture_frame.Distance: {picture_frame.Distance}')
            print(f'picture_frame.AmbientTemperature: {picture_frame.AmbientTemperature}')
            print(f'picture_frame.RelativeHumidity: {picture_frame.RelativeHumidity}')
            print(f'picture_frame.Height: {picture_frame.Height}')
            print(f'picture_frame.Width: {picture_frame.Width}')
            print(f'picture_frame.Precision: {picture_frame.Precision}')
            print(f'picture_frame.IRData: {picture_frame.IRData}')


            return picture_frame.IRData
        finally:

            self.heat_map.seek(0)

# 示例用法
fir_frame = get_fir_file("/home/yuan/server_ws/src/thermal_ds4025ft/scripts/HeatMap1.jpg")
# print("光學透過率:", fir_frame.OptiTrans)
# print("辐射率:", fir_frame.Emiss)
# print("拍攝距離:", fir_frame.Distance)
# print("環境溫度:", fir_frame.AmbientTemperature)
# print("相對濕度:", fir_frame.RelativeHumidity)
# print("圖像寬度:", fir_frame.Width)
# print("圖像高度:", fir_frame.Height)
# print("精度:", fir_frame.Precision)
# # np.set_printoptions(precision=5)

# reshaped_array = fir_frame.IRData.reshape(fir_frame.Height, fir_frame.Width)
# print(fir_frame.IRData)


