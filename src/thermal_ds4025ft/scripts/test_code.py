#!/usr/bin/env python3

import requests
import json
import hashlib
import re


URL_GetPTZStatus = "http://192.168.1.100:8080/api/v1/thermal"


def md5value(key) -> str:
    input_name = hashlib.md5()
    input_name.update(key.encode("utf-8"))
    return input_name.hexdigest().lower()


def IsLogin(*, url: str, account: str, password: str) -> bool:

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

        HA1 = md5value(f"{account}:{realm}:{password}")
        HA2 = md5value(f"GET:{url}")
        nc = "00000002"
        response = md5value(
            f"{HA1}:{nonce}:{nc}:{nonce}:{qop}:{HA2}"
        )
        authResponse = f'Digest username="{account}", realm="{realm}", nonce="{nonce}", uri="{url}", qop={qop}, nc={nc}, cnonce="{nonce}", response="{response}", opaque="{opaque}"'


        headers = {"Authorization": authResponse}
        response = requests.get(url, headers=headers)
        print(f"response: {response.text}")


        return False


def main():
    url = "http://192.168.1.108/cgi-bin/ptz.cgi?action=start&channel=1&code=Up&arg1=0&arg2=1&arg3=0"
    account = "admin"
    password = "admin"

    IsLogin(url=url, account=account, password=password)

    # if login(url=url):
    #     print("登入成功")
    # else:
    #     print("登入失敗")


if __name__ == "__main__":
    main()