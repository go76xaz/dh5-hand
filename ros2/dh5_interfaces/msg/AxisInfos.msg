std_msgs/Header header  # 标准头信息，包含时间戳
string[] name
uint32[] position
int32[] velocity
int32[] currents
uint32[] cur_faults
float64[] position_percent  # position feedback as percent of stroke, 0-100
uint8[] status              # per axis: 0 moving, 1 reached position, 2 stalled
