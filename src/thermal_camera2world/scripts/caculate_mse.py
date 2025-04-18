import cv2
import numpy as np
import glob
import os



# 指定儲存的 npz 路徑
device = 'coin417rg2'
npz_path = os.path.expanduser(f'~/calibration_data/{device}/exp2/calibration_data.npz')

# 讀取 npz 檔案
data = np.load(npz_path, allow_pickle=True)

# 取得每個變數
obj_points=data['obj_points']
img_points=data['img_points']
# ret=data['ret']
camera_matrix = data['camera_matrix']
dist_coeffs = data['dist_coeffs']
rvecs = data['rvecs']
tvecs = data['tvecs']

# 計算每張圖的 MSE
all_mse = []
for i in range(len(obj_points)):
    projected_points, _ = cv2.projectPoints(
        obj_points[i], rvecs[i], tvecs[i], camera_matrix, dist_coeffs
    )
    projected_points = projected_points.squeeze()
    img_pts = img_points[i].squeeze()

    mse = np.mean(np.sum((projected_points - img_pts)**2, axis=1))
    all_mse.append(mse)
    print(f"Image {i+1} MSE: {mse:.4f}")

overall_mse = np.mean(all_mse)
print(f"\n📌 Overall average MSE: {overall_mse:.4f}")
