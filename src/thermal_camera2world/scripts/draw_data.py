import numpy as np 
import matplotlib.pyplot as plt
import os


def circle_pattern():
    # 3D坐標點
    col = 3
    row = 11
    # 設定標定板尺寸（內部圓點數量）
    pattern_size = (col, row)  # 或者根據實際標定板調整


    obj_points = []  # 3D 世界座標
    img_points = []  # 2D 影像座標
    # 準備 3D 世界座標點
    objp = []
    for c in range(pattern_size[0]):
        for r in range(pattern_size[1]):
            if (r%2 == 0):
                objp.append([r, c*2, 0])
            else:
                objp.append([r, c*2+1, 0])

    objp = np.array(objp, np.float32)

    print(objp)

    index = np.lexsort((objp[:, 2], objp[:, 1], objp[:, 0]))

    # print(index)
    # 再按 y 排序
    # sorted_objp = sorted_by_x[np.argsort(sorted_by_x[:, 1])]

    # print(sorted_objp)
    ans = objp[index]
    print(ans)


    # # 畫出2D投影
    # plt.scatter(objp[:, 0], objp[:, 1], c='purple', marker='o')
    # plt.title("2D Projection of Points")
    # plt.xlabel("X")
    # plt.ylabel("Y")
    # plt.grid(True)
    # plt.show()




def UWB_to_pixel(device="IPT430M"):

    # 載入資料  
    pixel_path = os.path.expanduser(f"~/thermal_{device}_pixel_points.npy")
    world_path = os.path.expanduser(f"~/thermal_{device}_world_points.npy")

    # 載入資料
    pixel_points = np.load(pixel_path)
    world_points = np.load(world_path)

    # 畫布設置
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12, 5))

    # ===== 畫像素座標圖 =====
    ax1.set_title("Pixel Coordinates")
    ax1.scatter(pixel_points[:, 0], pixel_points[:, 1], c='red', label='pixel')
    ax1.set_xlabel("x (pixels)")
    ax1.set_ylabel("y (pixels)")
    ax1.invert_yaxis()  # 影像 y 軸通常是反的
    ax1.grid(True)
    ax1.legend()
    ax1.axis("equal")

    # ===== 畫世界座標圖 =====
    ax2.set_title("World Coordinates")
    ax2.scatter(world_points[:, 0], world_points[:, 1], c='blue', label='world')
    ax2.set_xlabel("x (world)")
    ax2.set_ylabel("y (world)")
    ax2.grid(True)
    ax2.legend()
    ax2.axis("equal")

    for i in range(len(pixel_points)):
        ax1.annotate(str(i), (pixel_points[i, 0], pixel_points[i, 1]))
        ax2.annotate(str(i), (world_points[i, 0], world_points[i, 1]))


    plt.tight_layout()
    plt.show()  



def read_matrix():

    # 指定儲存的 npz 路徑
    device = 'ds4025ft'
    npz_path = os.path.expanduser(f'~/calibration_data/{device}/exp3/calibration_data.npz')

    # 讀取 npz 檔案
    data = np.load(npz_path, allow_pickle=True)

    # 取得每個變數
    obj_points=data['obj_points']
    img_points=data['img_points']
    ret=data['ret']
    camera_matrix = data['camera_matrix']
    dist_coeffs = data['dist_coeffs']
    rvecs = data['rvecs']
    tvecs = data['tvecs']

    # print("📌 img_points:")
    # for i, img_point in enumerate(img_points):
    #     print(f"img_points[{i}]:\n{img_point}")

    print("📌ret:")
    print(ret)

    # 印出內容
    print("📌 相機內參矩陣 (camera_matrix):")
    print(camera_matrix)

    print("\n📌 畸變係數 (dist_coeffs):")
    print(dist_coeffs)

    print("\n📌 旋轉向量 (rvecs):")
    for i, rvec in enumerate(rvecs):
        print(f"rvec[{i}]:\n{rvec}")

    print("\n📌 平移向量 (tvecs):")
    for i, tvec in enumerate(tvecs):
        print(f"tvec[{i}]:\n{tvec}")




if __name__ == "__main__":
    # UWB_to_pixel("IPT430M")
    # UWB_to_pixel("DS4025FT")
    read_matrix()