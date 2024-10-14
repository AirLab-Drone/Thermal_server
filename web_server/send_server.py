import requests
import json

def send_data_to_server():
    url = 'http://127.0.0.1:5000/'  # 替換為您的 Flask 伺服器的 URL
    headers = {'Content-Type': 'application/json'}
    data = {
        'battery_remaining': 56.0,
        'voltage': 18,
        'current': 3.3
    }

    try:
        response = requests.post(url, headers=headers, data=json.dumps(data))
        if response.status_code == 200:
            print('Data sent successfully:', response.json())
        else:
            print('Failed to send data:', response.status_code, response.text)
    except requests.exceptions.RequestException as e:
        print(f'Error sending data: {e}')

if __name__ == '__main__':
    send_data_to_server()