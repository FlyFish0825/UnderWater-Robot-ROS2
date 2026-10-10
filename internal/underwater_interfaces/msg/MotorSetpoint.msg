# 推进器目标转速指令：由分配器发布，网关消费后转换为 CAN 帧下发。
# 约定：rpm 按 CAN 节点号-1 索引，单位 RPM，正负号表示转向；
# armed=false 表示停机指令，node_mask/run_mask 用于选择性寻址与状态诊断。
std_msgs/Header header  # 指令生成时刻与 wrench 参考坐标系
# Binds commands to the current serial connection, not a previous USB session.
uint64 connection_id  # 绑定当前串口连接，防止跨 USB 会话误用
# A new identifier is issued for every explicit enable/rearm.
uint64 session_id  # 每次显式 enable/rearm 递增的会话号
uint64 sequence  # 指令序号，逐条自增用于丢失检测
bool armed  # 是否处于武装状态，false 即停机
uint8 node_mask  # 参与寻址的 CAN 节点位掩码，bit0 对应节点 1
uint8 run_mask  # 非零转速通道位掩码，bit0 对应节点 1
# Indexed by CAN NodeID - 1, never geometry row after remapping.
int16[8] rpm  # 目标转速 (RPM)，按 CAN 节点号-1 索引，带正负号
