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


class Thermal_DS4025FT:
    def __init__(self, ip_address: str, account: str, password: str) -> None:
        self.__IRWidth = 384
        self.__IRHeight = 288
        self.__RGBWidth = 1280
        self.__RGBHeight = 1024

        self.ip_address = ip_address
        self.account = account
        self.password = password
        self.heat_map = None
        self.IRData = None

    def md5value(self, key) -> str:
        input_name = hashlib.md5()
        input_name.update(key.encode("utf-8"))
        return input_name.hexdigest().lower()

    def LoginThermalCamera(self, url: str) -> dict:

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
            response = self.md5value(f"{HA1}:{nonce}:{nc}:{nonce}:{qop}:{HA2}")
            authResponse = f'Digest username="{self.account}", realm="{realm}", nonce="{nonce}", uri="{url}", qop={qop}, nc={nc}, cnonce="{nonce}", response="{response}", opaque="{opaque}"'

            headers = {"Authorization": authResponse}

            return headers
        else:
            raise ValueError("Failed to login.")

    def getPointTemperature(self, x: int, y: int) -> float:
        """
        Get temperature at a pixel position on the thermal camera.
            :param x: int, x position of the point, [0-8191]
            :param y: int, y position of the point, [0-8191]
            :return: float, temperature at the point.
        """
        # x is [0-8191], y is [0-8191]
        if x < 0 or x > 8191 or y < 0 or y > 8191:
            raise ValueError("x or y out of range.")

        url = f"http://{self.ip_address}/cgi-bin/RadiometryManager.cgi?action=getRandomPointTemper&channel=2&coordinate[0]={x}&coordinate[1]={y}"
        header = self.LoginThermalCamera(url=url)

        if header:
            response = requests.get(url, headers=header)
            if response.status_code == 200:
                return response.text

        raise ValueError("Failed to get point temperature.")

    def setHeatMapFormat(self, format: str = "IR-SGCC-FIR64") -> bool:
        """
        DLT/664熱圖格式: "IR-SGCC",
        FIR熱圖格式: "IR-SGCC-FIR64",
        私有熱圖格式: "IR-SHEEN"
        """

        if format in ["IR-SGCC", "IR-SGCC-FIR64", "IR-SHEEN"]:
            url = f"http://{self.ip_address}/cgi-bin/configManager.cgi?action=setConfig&HeatImagingThermometry.HeatMapFormat={format}"
            header = self.LoginThermalCamera(url=url)
            if header:
                response = requests.get(url, headers=header)
                if response.text == "OK\n":
                    return True
        else:
            raise ValueError("Heat map format not supported.")

    def getHeatMap(self) -> None:
        """
        Get heat map from DS4025FT thermal camera.
        this function will save the heat map to self.heat_map
        """
        url = f"http://{self.ip_address}/cgi-bin/RPC_Loadfile/RadiometryHeatMap.jpg&channel=2"
        header = self.LoginThermalCamera(url=url)
        if header:
            response = requests.get(url, headers=header, stream=True)
            if response.status_code == 200:
                self.heat_map = io.BytesIO(response.raw.read())
            else:
                raise ValueError("Failed to get heat map.")

    def getHostTemperatureAndPosition(self) -> tuple:
        self.getHeatMap()

        # 移動文件指針到 IRData 的開始位置
        self.heat_map.seek(64)

        # 讀取溫度矩陣
        matrix_size = self.__IRWidth * self.__IRHeight
        self.IRData = struct.unpack(
            "h" * matrix_size, self.heat_map.read(2 * matrix_size)
        )
        # 把IRData轉成np.array
        self.IRData = np.array(self.IRData).reshape(self.__IRHeight, self.__IRWidth)
        max_temp = np.max(self.IRData)
        max_temp_position = np.unravel_index(
            np.argmax(self.IRData), self.IRData.shape
        )  # [y, x]
        max_temp_position = (max_temp_position[1], max_temp_position[0])  # [x, y]
        
        RGB_max_temp_position = (
            max_temp_position[0] / self.__IRWidth * self.__RGBWidth,
            max_temp_position[1] / self.__IRHeight * self.__RGBHeight,
        )  # [x, y]

        return max_temp, RGB_max_temp_position

    def getThermalStream(
        self, channel: int = 2, subtype: int = 0
    ) -> cv2.VideoCapture | bool:
        """
        Get thermal stream from DS4025FT thermal camera.
            :param channel: int, RGB: 1, IR: 2, default 2
            :param subtype: int, 主碼流: 0, 輔碼流: 1, default 0
            :return: cv2.VideoCapture|bool, cv2.VideoCapture if success, False if failed.
        """
        url_GetThermalStream = f"rtsp://{self.account}:{self.password}@{self.ip_address}/cam/realmonitor?channel={channel}&subtype={subtype}"

        vcap = cv2.VideoCapture(url_GetThermalStream)

        if vcap:
            return vcap
        else:
            raise ValueError("Failed to get thermal stream.")


def main():
    account = "admin"
    password = "admin"
    ip_address = "192.168.112.87"
    thermalCamera = Thermal_DS4025FT(
        ip_address=ip_address, account=account, password=password
    )
    # thermalCamera.setHeatMapFormat(format="IR-SGCC-FIR64")
    thermalCamera.getHeatMap()

    vcap = thermalCamera.getThermalStream()

    try:
        while True:

            print(thermalCamera.getHostTemperatureAndPosition())

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
