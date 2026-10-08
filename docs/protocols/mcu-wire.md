# MCU 接收协议依据

本版根据 2026-10-07 本机 Underwater_GUI 当前工作区实现核对：
- src/communication/protocol/CanGatewayProtocol.cpp
- SystemHeartbeatProtocol.cpp、SensorProtocol.cpp、ObserverMotorProtocol.cpp/.h
- docs/AA5B_Sensor_Protocol.md、docs/aa55-host-adaptation-20261007.md

该 GUI 工作区含未提交修改，本项目没有修改 GUI 或 MCU 固件。

## 帧族

- AA55：BODY_LEN 8..76，总长 BODY_LEN+6，55 AA 尾；
  CRC8 poly 07、init 00，覆盖 BODY_LEN 到最后包体字节。
  普通帧有 SEQ:u16、CAN_ID:u32、FLAGS:u8、LEN:u8 和 DATA。
  支持 8+N 旧紧凑、12+N 新时间戳、历史固定 72 字节包体。
  BODY=72、N=60 的固有歧义优先解释为新版。
  新时间戳紧随 DATA，不能被当作 CAN payload。
  FLAGS=80 的配置回复按独立布局处理，不当作电机数据。
- AA58：总长 20+N，N=0 或 5；CRC16-CCITT-FALSE，初值 FFFF，poly 1021。
  覆盖从第二个头字节起直到 payload 末尾，多字节与 CRC 均小端。
  当前五个字节分别表示输入、输出、CAN RX、CAN TX、流控队列占用率。
- AA5B：同类 20+N 包装，N<=58，VERSION=1，TARGET=1/2；
  FLAGS=1/2/6/8。0x80 原始 IMU、0x81 姿态、0x82 深度、0x83 状态。
  新增TARGET=1的0x85气压组：status:u32 + height/temp/pressure/reference四个f32，共20字节。
  需要配套固件转发；ROS不从0x80/0x81状态位猜测气压数值。
- AA59：按同类 20+N 包装校验并保留原始语义，最大 N=4096；本轮未实现流控发送。

单个有界增量解析器处理混合流；完整有效帧内的嵌套帧头不会被其他解析器二次识别。
坏候选从下一个字节重同步；半帧等待最多 1 秒，缓冲上限 65536 字节。
小端字段逐字节解码，浮点按 IEEE754 binary32 处理，不直接 memcpy 结构体。

## 已实现设备语义

IMU：g × 9.80665 → m/s²；gyro 原协议已经是 rad/s；磁场单位未确认。
深度：严格按有效位处理压力、温度、深度、水面参考及原始 ADC；不擅自换算未确认型号。
电机普通反馈 0x201..0x208：FD FLAGS=02/06，12 字节；
速度 i16 rpm，电流 u16×10/65535 A，电压 u16/100 V，
温度 -20+u16×170/65535 °C。心跳 0x281..0x288 另有内层 CRC8 与 NodeID 校验。

设备时间是单调微秒低 32 位，约 71.6 分钟回绕；不是 UTC。
时间高水位过滤假设相邻时间参考/重排距离小于半周期；MCU 不掉 USB 的重启不能保证自动识别。
长期中断或 MCU 重启后应重建连接。本版没有请求/回复时钟同步。

## 受限电机下行

手动模式显式 enable_motor_tx=true 后，采用旧紧凑 AA55（BODY_LEN=32，总长38），
CAN ID=0x100、FLAGS=02、24字节 payload；下行没有新增上行时间戳。
速度向量 0x10 更新八路有符号 rpm，运行向量 0x11 设置 RunMask。
停机必须发送 0x11 清运行位，0x10 的零速/零运行位不能替代它。
实现只允许 0 或 ±1000..6000 rpm，含会话、顺序、时间戳、连接和超时检查。
参见 docs/deployment/thrust-allocation.md；主机超时不能替代 MCU 断链看门狗。
