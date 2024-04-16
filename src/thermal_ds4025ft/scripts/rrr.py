
#!/usr/bin/env python3


import requests
from requests.auth import HTTPBasicAuth
import hashlib


# 計算第二次請求所需的密碼
username = "admin"
password = "admin"
realm = "Login to 02:31:67:82:61:43"
random = "17742056"

def md5value(key):
    input_name = hashlib.md5()
    input_name.update(key.encode('utf-8'))
    return input_name.hexdigest().upper()

# MD5(admin:17742056:MD5(admin:Login to 02:31:67:82:61:43:admin))


y = md5value('admin:17742056:' + md5value('admin:Login to 02:31:67:82:61:43:admin'))

print(y)