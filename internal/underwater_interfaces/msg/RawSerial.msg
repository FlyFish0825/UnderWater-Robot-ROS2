# 串口原始字节流：用于调试或外部工具复现协议解析，网关逻辑本身不依赖该话题。
# data 中的字节顺序与串口线上完全一致，按接收顺序拼接，不保证按帧对齐。
std_msgs/Header header  # 本批字节的接收时刻
uint64 connection_id  # 来源串口连接标识
uint8[] data  # 原始字节，长度可变
