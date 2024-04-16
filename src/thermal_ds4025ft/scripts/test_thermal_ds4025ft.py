#!/usr/bin/env python3


import requests
import hashlib
import random
import string

# Function to calculate MD5 hash
def md5(text):
    return hashlib.md5(text.encode()).hexdigest()

# Function to generate a random string of specified length
def generate_random_string(length):
    letters = string.ascii_letters + string.digits
    return ''.join(random.choice(letters) for i in range(length))

# Function to perform HTTP request with digest authentication
def perform_http_request_with_digest_auth(url, username, password, realm, nonce, opaque):
    # Generate cnonce
    cnonce = generate_random_string(16)
    
    # Construct HA1
    HA1 = md5(f"{username}:{realm}:{password}")
    
    # Construct HA2
    uri = url.split("://")[1].split("/", 1)[1]  # Extract URI from URL
    HA2 = md5(f"GET:{uri}")
    
    # Construct response
    nc = "00000002"
    qop = "auth"
    response = md5(f"{HA1}:{nonce}:{nc}:{cnonce}:{qop}:{HA2}")
    
    # Construct Authorization header
    auth_response = (
        f'Digest username="{username}", realm="{realm}", nonce="{nonce}", uri="{uri}", '
        f'qop={qop}, nc={nc}, cnonce="{cnonce}", response="{response}", opaque="{opaque}"'
    )
    
    # Perform HTTP GET request with Authorization header
    headers = {"Authorization": auth_response}
    response = requests.get(url, headers=headers)
    
    return response

# Example usage
url = "http://192.168.1.108/cgi-bin/configManager.cgi?action=getConfig&name=VideoWidget"
username = "admin"
password = "admin"
realm = "Login to GD0310PAZ00022"
nonce = "220711551"
opaque = "ecd96bfa32ed521cda6e9a8ed1701e6b3ef687d0"

response = perform_http_request_with_digest_auth(url, username, password, realm, nonce, opaque)

print("HTTP Status Code:", response.status_code)
print("Response Body:", response.text)
