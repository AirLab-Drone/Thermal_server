#!/usr/bin/env python3

from datetime import datetime, timezone, timedelta
import json
import requests
import os




def post_to_server(url=None, file=None, data=None, json=None):
    """
    @param url: url
    @param file: file
    @param data: data
    @return: response
    """
    return requests.post(url, files=file, data=data, json=json)


def print_json(data):
    """
    @param data: data
    """
    print(json.dumps(data, indent=4))


def check_port_exists(port):
    return os.path.exists(port)


def ros2_time_to_taiwan_timezone(ros2_time) -> datetime:
    """
    將 ROS 2 時間轉換為台灣時區的 datetime
    @param ros2_time: ROS 2 時間
    @return: 台灣時區的 datetime
    """
    # 將 ROS 2 時間轉換為 UTC 的 datetime
    current_time_msg = ros2_time.to_msg()
    utc_datetime = datetime.fromtimestamp(
        current_time_msg.sec + current_time_msg.nanosec / 1e9, tz=timezone.utc
    )

    # 定義台灣時區 (UTC+8)
    taiwan_timezone = timezone(timedelta(hours=8))
    
    # 轉換為台灣時區
    taiwan_datetime = utc_datetime.astimezone(taiwan_timezone)
    
    return taiwan_datetime