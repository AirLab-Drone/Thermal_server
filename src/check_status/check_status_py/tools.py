import requests



def send_json_to_database(url, data):
    """
    @param url: url
    @param data: json data
    @return: response
    """
    url = ""
    return requests.post(url, json=data)


def send_json_to_server(url, data):
    # print(f"send_json_to_server {url}")
    print(data)
    pass