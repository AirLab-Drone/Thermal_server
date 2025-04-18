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



# 計算每張圖的 MRE
all_mre = []
for i in range(len(obj_points)):
    # 使用相機參數將 3D 世界座標投影為 2D 圖像點
    projected_points, _ = cv2.projectPoints(
        obj_points[i], rvecs[i], tvecs[i], camera_matrix, dist_coeffs
    )
    projected_points = projected_points.squeeze()
    img_pts = img_points[i].squeeze()

    # 計算每個點的相對誤差（相對誤差 = |預測點 - 實際點| / |實際點|）
    relative_errors = np.linalg.norm(projected_points - img_pts, axis=1) / np.linalg.norm(img_pts, axis=1)

    # 找出最大的相對誤差（MRE）
    mre = np.max(relative_errors)
    all_mre.append(mre)
    print(f"Image {i+1} MRE: {mre:.4f}")

# 計算所有圖片的平均 MRE
overall_mre = np.mean(all_mre)
print(f"\n📌 Overall average MRE: {overall_mre:.4f}")
