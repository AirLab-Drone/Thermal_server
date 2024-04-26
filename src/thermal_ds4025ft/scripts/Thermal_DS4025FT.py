#!/usr/bin/env python3

import requests
import json
import hashlib
import re
import time
import cv2 
import numpy as np
import struct
import io

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

class Thermal_DS4025FT():
    def __init__(self, ip_address: str, account: str, password: str) -> None:
        self.ip_address = ip_address
        self.account = account
        self.password = password
        self.heat_map = None
        self.picture_frame = FirFrame()

        
    def md5value(self, key) -> str:
        input_name = hashlib.md5()
        input_name.update(key.encode("utf-8"))
        return input_name.hexdigest().lower()


    def login_thermal_camera(self, url: str) -> dict|bool:

        login_response = requests.get(url)

        if login_response.headers:

            login_info = login_response.headers.get("WWW-Authenticate", "")
            # print(f"login_response.headers: {login_response.headers}")
            # print(f"login_info: {login_info}")

            login_info_list = re.findall(
                r'=(?:")(.*?)(?:")', login_info
            )  # [realm, qop, nonce, opaque]

            realm = login_info_list[0]
            qop = login_info_list[1]
            nonce = login_info_list[2]
            opaque = login_info_list[3]

            HA1 = self.md5value(f"{self.account}:{realm}:{self.password}")
            HA2 = self.md5value(f"GET:{url}")
            nc = "00000002"
            response = self.md5value(
                f"{HA1}:{nonce}:{nc}:{nonce}:{qop}:{HA2}"
            )
            authResponse = f'Digest username="{self.account}", realm="{realm}", nonce="{nonce}", uri="{url}", qop={qop}, nc={nc}, cnonce="{nonce}", response="{response}", opaque="{opaque}"'

            headers = {"Authorization": authResponse}

            return headers
        else:
            return False
        

    def getPointTemperature(self, x:int, y:int) -> float|bool:
        # x is [0-8191], y is [0-8191]
        if x < 0 or x > 8191 or y < 0 or y > 8191:
            return False
        url = f'http://{self.ip_address}/cgi-bin/RadiometryManager.cgi?action=getRandomPointTemper&channel=2&coordinate[0]={x}&coordinate[1]={y}'
        header = self.login_thermal_camera(url=url)
        if header:
            response = requests.get(url, headers=header)
            if response.status_code == 200:
                
                return response.text


    def setHeatMapFormat(self, format: str = "IR-SGCC-FIR64") -> bool:
        """
        DLT/664熱圖格式: "IR-SGCC", 
        FIR熱圖格式: "IR-SGCC-FIR64", 
        私有熱圖格式: "IR-SHEEN"
        """

        if format in ["IR-SGCC", "IR-SGCC-FIR64", "IR-SHEEN"]:
            url = f'http://{self.ip_address}/cgi-bin/configManager.cgi?action=setConfig&HeatImagingThermometry.HeatMapFormat={format}'
            header = self.login_thermal_camera(url=url)
            if header:
                response = requests.get(url, headers=header)
                if response.text == "OK\n":
                    return True
        else:
            raise ValueError("Heat map format not supported.")
            
    
    def getHeatMap(self) -> None:
        url = f'http://{self.ip_address}/cgi-bin/RPC_Loadfile/RadiometryHeatMap.jpg&channel=2'
        header = self.login_thermal_camera(url=url)
        if header:
            response = requests.get(url, headers=header, stream=True).raw
            # TODO: response有時不會有？
            if response.status_code == 200:
                self.heat_map = io.BytesIO(response.read())
            
                
            

    def getTemperatureMartix(self) -> np.array:
            
            
            try:
            
                # 移動文件指針到 IRData 的開始位置
                self.heat_map.seek(64)

                # 讀取溫度矩陣
                matrix_size = self.picture_frame.Width * self.picture_frame.Height
                self.picture_frame.IRData = struct.unpack('h' * matrix_size, self.heat_map.read(2 * matrix_size))
                # 把IRData轉成np.array
                self.picture_frame.IRData = np.array(self.picture_frame.IRData).reshape(self.picture_frame.Height, self.picture_frame.Width)
                self.picture_frame.IRData = self.picture_frame.IRData / 10.0
                print(self.picture_frame.IRData)
            
                return self.picture_frame.IRData
            finally:
                
                self.heat_map.seek(0)
                
            
        
        
    def getThermalStream(self, channel: int = 2, subtype: int = 0) -> cv2.VideoCapture|bool:
        """
        Get thermal stream from DS4025FT thermal camera.
            :param channel: int, RGB: 1, IR: 2, default 2
            :param subtype: int, 主碼流: 0, 輔碼流: 1, default 0
            :return: cv2.VideoCapture|bool, cv2.VideoCapture if success, False if failed.
        """
        url_GetThermalStream = f'rtsp://{self.account}:{self.password}@{self.ip_address}/cam/realmonitor?channel={channel}&subtype={subtype}'
        
        vcap  = cv2.VideoCapture(url_GetThermalStream)
        
        if vcap:
            return vcap
        else:
            raise ValueError("Failed to get thermal stream.")

        


        


    




def main():
    account = "admin"
    password = "admin"
    ip_address = "192.168.1.108"
    thermalCamera = Thermal_DS4025FT(ip_address=ip_address, account=account, password=password)
    # thermalCamera.setHeatMapFormat(format="IR-SGCC-FIR64")
    # thermalCamera.getHeatMap()

    vcap = thermalCamera.getThermalStream()

    try:
        while True:
            thermalCamera.getHeatMap()
            thermalCamera.getTemperatureMartix()
            
            # ret, frame = vcap.read()    
            # if ret:
            #     cv2.imshow('VIDEO', frame)

            # key = cv2.waitKey(1)
            # if key == 27:
            #     break
    finally:
        pass
        # vcap.release()
        # cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
