import numpy as np 
import matplotlib.pyplot as plt

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

