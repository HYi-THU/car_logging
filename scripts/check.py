import pandas as pd

# 假设数据文件名为2025-01-06_12-03-12.txt，并且已经保存在了正确的路径
file_path = '../data/2025-01-06_12-03-12.txt'

# 初始化空列表来存储数据
data = []

# 打开并读取文件
with open(file_path, 'r') as file:
    lines = file.readlines()
    cnt = 0
    last_gps_time = 0
    imu_frequency = 1
    for line in lines:
        if line.strip():  # 确保不处理空行
            parts = line.split(',')
            if parts[0][0:5] == 'GpsTm':
                gps_time = parts[0][6:]
                gps_time_int = int(parts[0][6:])
                gps_interval = gps_time_int - last_gps_time

                # 将解析的数据添加到列表中
                if len(data) == 0:
                    data.append([gps_time_int, '--', '----', '----'])
                else:
                    if gps_interval > 0:
                        imu_frequency = cnt / gps_interval
                    else:
                        imu_frequency = 888888                            
                    data.append([gps_time_int, gps_interval, cnt, imu_frequency])

                last_gps_time = gps_time_int
                cnt = 0
            else:
                cnt += 1

# 创建DataFrame
df = pd.DataFrame(data, columns=['GPS时间', 'GPS间隔时间', 'GPS间隔期的IMU数量', 'IMU的频率'])

# 将DataFrame保存为TXT文件，实际上是一个CSV格式的文本文件
df.to_csv('check_result.txt', index=False, sep='\t')