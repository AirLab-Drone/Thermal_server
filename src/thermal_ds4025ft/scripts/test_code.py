#!/usr/bin/env python3

import requests
import json
import hashlib
import re
import time
import cv2 
import numpy as np


URL_GetPTZStatus = "http://192.168.1.100:8080/api/v1/thermal"


class Thermal_DS4025FT():
    def __init__(self, ip_address: str, account: str, password: str) -> None:
        self.ip_address = ip_address
        self.account = account
        self.password = password
        
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
        
    # TODO: add more functions to get thermal camera HeatMap, get point temperature, set PTZ, etc.

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

    def getHeatMap(self, url: str) -> np.ndarray:
        
        pass


# def md5value(key) -> str:
#     input_name = hashlib.md5()
#     input_name.update(key.encode("utf-8"))
#     return input_name.hexdigest().lower()




        


def GetThermalStream(account: str = 'admin', 
                     password: str = 'admin',
                     ip_address: str = '192.168.1.108',
                     channel: int = 2, 
                     subtype: int = 0) -> cv2.VideoCapture|bool:
    """
    Get thermal stream from DS4025FT thermal camera.
        :param account: str, default 'admin'
        :param password: str, default 'admin'
        :param ip_address: str, default '192.168.1.108'
        :param channel: int, RGB: 1, IR: 2, default 2
        :param subtype: int, 主碼流: 0, 輔碼流: 1, default 0
        :return: cv2.VideoCapture|bool, cv2.VideoCapture if success, False if failed.
    """

    url_GetThermalStream = f'rtsp://{account}:{password}@{ip_address}/cam/realmonitor?channel={channel}&subtype={subtype}'

    vcap  = cv2.VideoCapture(url_GetThermalStream)

    if vcap:
        return vcap
    else:
        return False
    

def GetThermalHeatMap():
    '''
    切换为国网DLT/664热图格式：
    http://192.168.1.108/cgi-bin/configManager.cgi?action=setConfig&HeatImagingThermometry.HeatMapFormat=IR-SGCC

    切换为国网FIR热图格式：
    http://192.168.1.108/cgi-bin/configManager.cgi?action=setConfig&HeatImagingThermometry.HeatMapFormat= IR-SGCC-FIR64

    切换为私有热图格式：
    http://192.168.1.108/cgi-bin/configManager.cgi?action=setConfig&HeatImagingThermometry.HeatMapFormat= IR-SHEEN
    '''

    pass



def main():
    url_gettHeatMap = 'http://192.168.1.108/cgi-bin/RPC_Loadfile/RadiometryHeatMap.jpg&channel=2'
    url_getPointTemp = 'http://192.168.1.108/cgi-bin/RadiometryManager.cgi?action=getRandomPointTemper&channel=2&coordinate[0]=0&coordinate[1]=0'

    account = "admin"
    password = "admin"
    ip_address = "192.168.1.108"

    # vcap = GetThermalStream(account=account, password=password, ip_address=ip_address)
    # IR frame shape: (1024, 1280, 3)
    # RGB frame shape: (1520, 2688, 3)

    # while True:

    #     ret, frame = vcap.read()    
    #     sframe=cv2.resize(frame,(1110, 890))
    #     cv2.imshow('VIDEO', sframe)

    #     key = cv2.waitKey(1)
    #     if key == 27:
    #         break

    thermalCamera = Thermal_DS4025FT(ip_address=ip_address, account=account, password=password)

    # while True:
    #     a = thermalCamera.getPointTemperature(url_getPointTemp)
    #     # print(type(a))
    #     print(a)

    loginHeader = thermalCamera.login_thermal_camera(url=url_gettHeatMap)

    response = requests.get(url_gettHeatMap, headers=loginHeader, stream=True).raw
    # image = np.asarray(bytearray(response.read()), dtype="uint8")
    print(type(bytearray(response.read())))
    # image = cv2.imdecode(image, cv2.IMREAD_COLOR)

    # for testing
    # cv2.imshow('image',image)
    # cv2.waitKey(0)

    # print(f"response.headers: {response.headers}")
    # print(f"response: {response.text}")



if __name__ == "__main__":
    main()
    # vcap = GetThermalStream()

    # while True:

    #     ret, frame = vcap.read()    
    #     sframe=cv2.resize(frame,(1110, 890))
    #     cv2.imshow('VIDEO', sframe)

    #     key = cv2.waitKey(1)
    #     if key == 27:
    #         break