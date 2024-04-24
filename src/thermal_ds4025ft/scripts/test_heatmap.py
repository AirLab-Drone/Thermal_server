from PIL import Image
import numpy as np

def read_jpg_image(file_path):
    # 使用 Pillow 讀取 JPG 圖像
    img = Image.open(file_path)
    
    # 將圖像轉換成 NumPy 數組
    img_array = np.array(img)
    
    return img_array

# 測試讀取 JPG 圖像
file_path = 'src/thermal_ds4025ft/scripts/HeatMap_2024-04-24_22-24-46_HeatMap.jpg'
image_data = read_jpg_image(file_path)
print("圖像數據:")
print(image_data)
