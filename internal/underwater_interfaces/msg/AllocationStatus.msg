# 分配器运行状态：诊断武装状态、求解结果与 RPM 量化副作用。
# 所有 wrench 为 NED 下六维量 [Fx Fy Fz] (N) 与 [Mx My Mz] (N·m)。
# 索引约定：force 数组按几何行顺序(8 个推进器)，rpm 数组按 CAN 节点号-1 排序。
std_msgs/Header header  # 宿主发布时刻与 wrench 参考坐标系
bool armed  # 是否已武装（允许下发非零转速）
bool input_fresh  # wrench 输入是否仍在超时窗口内
bool gateway_ready  # 网关是否已连接、已稳定且 tx_enabled
bool solver_ok  # 最近一次分配求解是否成功
bool rpm_deadband_applied  # 是否有通道因低于 RPM 死区被置零
bool rpm_saturated  # 是否有通道被钳位到 max_rpm
uint64 session_id  # 本次武装的会话号，用于与网关握手
float64[6] requested_wrench  # 求解器实际使用的请求 wrench
float64[6] allocated_wrench  # 求解推力对应的六维 wrench (N, N·m)
# Prediction using the temporary linear curve AFTER deadband, not measured wrench.
float64[6] modeled_rpm_wrench  # 经死区/量化后转速反推的等效 wrench，预测值而非实测
float64[8] thruster_force_n  # 各推进器模型推力 (N)，几何行顺序，可正可负
int16[8] proposed_motor_rpm  # 下发目标转速 (RPM)，按 CAN 节点号索引，带正负号
string detail  # 人类可读的状态说明或停机原因
