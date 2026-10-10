# Interface retained; no camera implementation or publisher yet.
# msg_pkg 保留接口（当前无实现、无发布者）：目标在相机光学坐标系下的位置与方向。
float32 center_x  # 目标中心 x（像素）
float32 center_y  # 目标中心 y（像素）
float32 center_z  # 目标中心 z（米）
float32 vector_x  # 方向向量 x 分量（归一化）
float32 vector_y  # 方向向量 y 分量（归一化）
float32 vector_z  # 方向向量 z 分量（归一化）
