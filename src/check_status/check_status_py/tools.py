import requests
import os


def send_json_to_database(url, data):
    """
    @param url: url
    @param data: json data
    @return: response
    """
    url = ""
    return requests.post(url, json=data)


# TODO:上傳失敗的錯誤處理
def send_json_to_server(url, data):
    # print(f"send_json_to_server {url}")
    print(data)
    pass


def check_port_exists(port):
    return os.path.exists(port)