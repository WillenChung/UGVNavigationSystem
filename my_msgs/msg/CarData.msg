int64 seq
string timestamp #世界时间(UTC) + 8小时 = 北京时间(BTC)
float32 longitude #[单位]度
float32 latitude #[单位]度
float32 spd_over_grnd #[单位]m/s
float32 local_x #只有当Lego LOAM在地图原点开始运行才有意义
float32 local_y
float32 real_yaw
