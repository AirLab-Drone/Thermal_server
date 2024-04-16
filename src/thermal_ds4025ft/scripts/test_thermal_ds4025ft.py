#!/usr/bin/env python3


import requests
from requests.auth import HTTPBasicAuth
import hashlib

import json


# 定義首次登錄的RPC請求參數
rpc_first_request = {
    "method": "global.login",
    "params": {
        "userName": "admin",
        "password": "",
        "clientType": "Web3.0",
        "loginType": "Direct",
    },
    "id": 1,
}
headers = {'user-agent': 'Mozilla/5.0 (X11; Linux x86_64) AppleWebKit/537.36 (KHTML, like Gecko) Chrome/123.0.0.0 Safari/537.36'}



# 發送首次RPC請求
response_first = requests.post(
    "http://192.168.1.108/RPC2_Login", json=rpc_first_request, headers=headers
)

# 檢查第一次請求的響應
if response_first.status_code == 200:

    # 獲取第一次請求的響應內容
    response_first_data = response_first.json()
    response_first_data_print = json.dumps(response_first_data, indent=4)
    print("第一次請求響應")
    print(response_first_data_print)
    print()

    # 獲取 realm 和 random 的值
    realm = response_first_data["params"]["realm"]
    random = response_first_data["params"]["random"]
    print("realm:", realm)
    print("random:", random)
    
    # 計算第二次請求所需的密碼
    username = 'admin'
    password = 'admin'  # 實際使用時，可能需要從安全的地方獲取密碼

    # MD5(username:random:MD5(username:realm:password))。
    hashed_password = md5value(username + ':' + random + ':' + md5value(username + ':' + realm + ':' + password))

    # 定義第二次登錄的RPC請求參數
    rpc_second_request = {
        "method": "global.login",
        "params": {
            "userName": "admin",
            "password": hashed_password,
            "clientType": "Web3.0",
            "loginType": "Direct",
            "authorityType": "Default",
        },
        "id": 2,
        "session": response_first_data["session"],
    }

    # 發送第二次RPC請求
    response_second = requests.post(
        "http://192.168.1.108/RPC2_Login", json=rpc_second_request, headers=headers
    )

    # 檢查第二次請求的響應
    if response_second.status_code == 200:
        response_second_data = response_second.json()
        response_second_data_print = json.dumps(response_second_data, indent=4)
        print("第二次請求響應")
        print(response_second_data_print)
        print()

        url = 'http://192.168.1.108/cgi-bin/RPC_Loadfile/RadiometryHeatMap.jpg&channel=2'
        # # url1 = 'http://admin:admin@192.168.1.108/cgi-bin/RPC_Loadfile/RadiometryHeatMap.jpg&channel=2'

        user = 'admin'
        password = 'admin'
        img = requests.get(url=url, auth=(user, password))
        # img = requests.get(url=url)

        print(img.status_code)
        


    else:
        print("登錄失敗")
else:
    print("第一次請求失敗")
