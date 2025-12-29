import machine
 
import ujson
import time
 
import struct
import neopixel
from machine import Pin, UART, I2C

TUYA_PID = "b7zdedszp3gq3k9t" # 涂鸦产品ID

##############################################################
### 配置常量区域 - 时间调度周期配置（单位：毫秒）
##############################################################
# 数据上报周期
CONFIG_REPORT_INTERVAL_MS = 30000        # 30秒上报一次数据到涂鸦平台

# 传感器采样周期
CONFIG_SENSOR_SAMPLE_INTERVAL_MS = 5000  # 5秒采样一次温湿度

# RS485状态轮询周期
CONFIG_RS485_POLL_INTERVAL_MS = 2000     # 2秒轮询一次风机状态

# 网络状态查询周期
CONFIG_NETWORK_QUERY_INTERVAL_MS = 5000  # 5秒查询一次网络状态

# LED更新周期
# 主循环周期
CONFIG_MAIN_LOOP_INTERVAL_MS = 10        # 10ms主循环周期

# RS485通信超时
CONFIG_RS485_TIMEOUT_MS = 200            # RS485读取超时200ms

# RS485命令重试
CONFIG_RS485_RETRY_COUNT = 3             # RS485命令重试次数
CONFIG_RS485_RETRY_DELAY_MS = 100        # 重试延迟100ms

# 按钮消抖
CONFIG_BUTTON_DEBOUNCE_MS = 50           # 按钮消抖时间50ms
CONFIG_BUTTON_LONG_PRESS_MS = 5000       # 长按5秒触发配网

##############################################################
### Modbus RTU 寄存器定义
##############################################################
MODBUS_DEV_ADDR = 0xA1                   # 风机从站地址
MODBUS_READ_FUNC = 0x03                  # 读保持寄存器功能码
MODBUS_WRITE_FUNC = 0x06                 # 写单个寄存器功能码
MODBUS_EXCEPTION_FUNC = 0x83             # 读寄存器异常响应功能码

# 寄存器地址定义（16位，大端序）
REG_POWER = 0x1005                       # 开关寄存器地址
                                        # 数据类型: uint16
                                        # 范围: 0=关闭, 1=开启
                                        # 行为: 写入0关闭风机，写入1开启风机

REG_SPEED = 0x1007                       # 档位寄存器地址
                                        # 数据类型: uint16
                                        # 范围: 1~4
                                        # 行为: 写入1~4设置对应档位

REG_STATUS_QUERY_START = 0x1005          # 状态查询起始地址
REG_STATUS_QUERY_COUNT = 3               # 状态查询寄存器数量（开关、保留、速度）
REG_STATUS_RESPONSE_BYTES = 6            # 状态查询应答数据字节数（3个寄存器*2字节）

##############################################################
### 日志系统
##############################################################
# 日志等级
LOG_LEVEL_DEBUG = 0
LOG_LEVEL_INFO = 1
LOG_LEVEL_WARN = 2
LOG_LEVEL_ERROR = 3

# 当前日志等级（可通过DP或编译开关调整）
LOG_LEVEL = LOG_LEVEL_INFO

# 错误码定义
ERR_OK = "OK"
ERR_RS485_TIMEOUT = "E485T"              # RS485超时
ERR_RS485_CRC = "E485C"                  # RS485 CRC错误
ERR_RS485_LEN = "E485L"                  # RS485长度错误
ERR_RS485_ADDR = "E485A"                 # RS485地址错误
ERR_RS485_FUNC = "E485F"                 # RS485功能码错误
ERR_RS485_WRITE_FAIL = "E485W"           # RS485写入失败
ERR_SENSOR_READ = "ESEN"                 # 传感器读取失败
ERR_TUYA_FRAME = "ETYA"                  # 涂鸦帧错误

# 统计计数器
stats_rs485_crc_errors = 0               # RS485 CRC错误计数
stats_rs485_len_errors = 0                # RS485长度错误计数
stats_rs485_timeouts = 0                 # RS485超时计数
stats_rs485_success = 0                  # RS485成功计数

def log(level, msg, err_code=None):
    """日志输出函数，减少字符串拼接开销"""
    if level < LOG_LEVEL:
        return
    
    if err_code:
        print(f"[{err_code}]", end=" ")
    print(msg)

def log_debug(msg):
    log(LOG_LEVEL_DEBUG, msg)

def log_info(msg):
    log(LOG_LEVEL_INFO, msg)

def log_warn(msg, err_code=None):
    log(LOG_LEVEL_WARN, msg, err_code)

def log_error(msg, err_code=None):
    log(LOG_LEVEL_ERROR, msg, err_code)

##############################################################
### 硬件初始化
##############################################################
uart = UART(0, baudrate=115200, tx=Pin(0), rx=Pin(1))  # 涂鸦模块通信端口，UART0使用GP0和GP1
rs485 = UART(1, baudrate=9600, tx=Pin(8), rx=Pin(9))   # 485接口使用UART1，UART1使用GP8和GP9

led = Pin(25, Pin.OUT)  # 模块25脚LED，网络状态LED
np = neopixel.NeoPixel(machine.Pin(16), 1) # GPIO 16连接WS2812，风速状态LED

# AHT20传感器
i2c = I2C(1, scl=Pin(7), sda=Pin(6), freq=100000)
AHT20_ADDR = 0x38

reset = Pin(28, Pin.IN, Pin.PULL_UP) # 模块28脚作为配网按钮

# 状态常量
STATE_UNPROVISIONED = 0
STATE_PROVISIONED = 1
STATE_CLOUD_CONNECTED = 2

# 全局变量
first_heartbeat = True       # 首次心跳请求
product_info_queried = False # 产品信息是否已查询

###########################################
###        环境和设备状态变量           #####
last_temp = 0                 #室内温度
last_humidity = 0             #室内湿度

fan_onoff = False              #风机状态，启动/关闭
fan_speed = 1                 #风速，1~4档
fan_status = True             #风机通信状态，是否响应指令
pending_speed = None          #待机时缓存的速度设置（关机状态下设置速度时缓存）

# 时间节拍器（使用ticks_ms确保溢出安全）
timer_report = 0              # 数据上报节拍器
timer_sensor = 0              # 传感器采样节拍器
timer_rs485_poll = 0          # RS485轮询节拍器
timer_network_query = 0      # 网络查询节拍器
timer_led = 0                 # LED更新节拍器
timer_button = 0              # 按钮检测节拍器
timer_button_press_start = 0  # 按钮按下开始时间
timer_rgb_blink = 0           # RGB闪烁节拍器

led_state = False
rgb_blink_state = False       # RGB闪烁状态

#############################################################
### AHT20 温湿度获取部分
#############################################################
def aht20_trigger_measurement():
    # 触发一次测量
    try:
        i2c.writeto(AHT20_ADDR, bytes([0xAC, 0x33, 0x00]))
        time.sleep(0.08)  # 等待测量完成
    except Exception as e:
        log_error("AHT20触发失败", ERR_SENSOR_READ)

def aht20_read_data():
    try:
        data = i2c.readfrom(AHT20_ADDR, 7)
        if (data[0] & 0x80) == 0:  # 检查状态位，确保数据有效
            raw_humi = ((data[1] << 12) | (data[2] << 4) | (data[3] >> 4)) & 0xFFFFF
            raw_temp = ((data[3] & 0x0F) << 16) | (data[4] << 8) | data[5]
            humidity = (raw_humi / 1048576.0) * 100
            temperature = (raw_temp / 1048576.0) * 200 - 50
            return temperature, humidity
        else:
            log_warn("AHT20数据无效", ERR_SENSOR_READ)
            return None, None
    except Exception as e:
        log_error("AHT20读取异常", ERR_SENSOR_READ)
        return None, None

def update_env_data():
    global last_temp, last_humidity, timer_sensor
    
    last_temp, last_humidity = aht20_read_data()
    if last_temp is not None and last_humidity is not None:
        log_debug(f"T:{last_temp:.1f} H:{last_humidity:.1f}")
    
    timer_sensor = time.ticks_ms()

#########################################################
### RGB灯变化 - 仅显示风机状态
### 通信异常：红色闪烁（500ms周期）
### 通信正常且风机开启：根据风速显示颜色（1档蓝色，2档绿色，3档橙色，4档红色）
### 通信正常且风机关闭：不亮
#########################################################
def set_rgb(level):
    global np, fan_onoff, fan_status, timer_rgb_blink, rgb_blink_state
    
    current_time = time.ticks_ms()
    
    # 通信异常：红色闪烁
    if not fan_status:
        # 500ms闪烁周期
        if time.ticks_diff(current_time, timer_rgb_blink) >= 500:
            rgb_blink_state = not rgb_blink_state
            timer_rgb_blink = current_time
        
        if rgb_blink_state:
            np.fill((32, 0, 0))  # 红色
        else:
            np.fill((0, 0, 0))   # 关闭
        np.write()
        return
    
    # 通信正常：根据风机状态和风速显示
    if fan_onoff:
        # 风机开启，根据风速显示颜色
        if level == 1:
            np.fill((0, 0, 16))      # 1档 蓝色
        elif level == 2:
            np.fill((8, 32, 0))      # 2档 绿色
        elif level == 3:
            np.fill((32, 16, 0))     # 3档 橙色
        elif level == 4:
            np.fill((32, 0, 0))      # 4档 红色
        else:
            np.fill((0, 0, 0))
    else:
        # 风机关闭，不亮
        np.fill((0, 0, 0))
    
    np.write()

def bytes_to_hex(byte_data):
    """将字节数据转换为十六进制字符串"""
    return ''.join('{:02X} '.format(b) for b in byte_data)

##############################################################
### Modbus RTU/RS485 部分
##############################################################
def modbus_crc16(data):
    crc = 0xFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            if crc & 0x0001:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
    return crc & 0xFFFF

def build_modbus_read(addr, start_reg, quantity):
    frame = bytearray()
    frame.append(addr)
    frame.append(MODBUS_READ_FUNC)
    frame.extend(struct.pack('>H', start_reg))
    frame.extend(struct.pack('>H', quantity))
    crc = modbus_crc16(frame)
    frame.append(crc & 0xFF)
    frame.append((crc >> 8) & 0xFF)
    return frame

def build_modbus_write_single(addr, reg, value):
    frame = bytearray()
    frame.append(addr)
    frame.append(MODBUS_WRITE_FUNC)
    frame.extend(struct.pack('>H', reg))
    frame.extend(struct.pack('>H', value))
    crc = modbus_crc16(frame)
    frame.append(crc & 0xFF)
    frame.append((crc >> 8) & 0xFF)
    return frame

def rs485_write(frame):
    try:
        log_debug("RS485 TX: " + bytes_to_hex(frame))
        rs485.write(frame)
    except Exception as e:
        log_error("RS485写入异常", ERR_RS485_WRITE_FAIL)

def rs485_read(timeout_ms=CONFIG_RS485_TIMEOUT_MS):
    start = time.ticks_ms()
    buf = bytearray()
    while time.ticks_diff(time.ticks_ms(), start) < timeout_ms:
        if rs485.any():
            chunk = rs485.read()
            if chunk:
                buf.extend(chunk)
        else:
            time.sleep_ms(5)
    if len(buf) > 0:
        log_debug("RS485 RX: " + bytes_to_hex(buf))
        return bytes(buf)
    # 超时无数据
    global stats_rs485_timeouts
    stats_rs485_timeouts += 1
    return None

def parse_modbus_read_response(resp, expected_addr, expected_func, expected_reg_count):
    """解析Modbus读响应，验证地址、功能码、数据长度和CRC"""
    global stats_rs485_crc_errors, stats_rs485_len_errors, stats_rs485_success
    
    if not resp or len(resp) < 5:
        stats_rs485_len_errors += 1
        log_warn("RS485响应太短", ERR_RS485_LEN)
        return None
    
    # 检查异常响应
    if resp[1] == (expected_func | 0x80):
        log_warn("RS485异常响应", ERR_RS485_FUNC)
        return None
    
    # 检查地址
    if resp[0] != expected_addr:
        log_warn("RS485地址错误", ERR_RS485_ADDR)
        return None
    
    # 检查功能码
    if resp[1] != expected_func:
        log_warn("RS485功能码错误", ERR_RS485_FUNC)
        return None
    
    # 检查字节数（寄存器数量 * 2）
    byte_count = resp[2]
    expected_bytes = expected_reg_count * 2
    if byte_count != expected_bytes:
        stats_rs485_len_errors += 1
        log_warn(f"RS485字节数错误:期望{expected_bytes}实际{byte_count}", ERR_RS485_LEN)
        return None
    
    # 校验CRC
    payload = resp[:-2]
    crc_recv = resp[-2] | (resp[-1] << 8)
    crc_calc = modbus_crc16(payload)
    if crc_recv != crc_calc:
        stats_rs485_crc_errors += 1
        log_warn("RS485 CRC错误", ERR_RS485_CRC)
        return None
    
    stats_rs485_success += 1
    return resp[3:3+byte_count]

def verify_modbus_write_response(resp, expected_addr, expected_reg, expected_value):
    """验证Modbus写响应"""
    global stats_rs485_crc_errors, stats_rs485_len_errors
    
    if not resp or len(resp) < 8:
        stats_rs485_len_errors += 1
        return False
    
    # 检查异常响应
    if resp[1] == (MODBUS_WRITE_FUNC | 0x80):
        log_warn("RS485写异常响应", ERR_RS485_FUNC)
        return False
    
    # 检查地址和功能码
    if resp[0] != expected_addr or resp[1] != MODBUS_WRITE_FUNC:
        return False
    
    # 检查寄存器地址和值
    reg_recv = (resp[2] << 8) | resp[3]
    value_recv = (resp[4] << 8) | resp[5]
    if reg_recv != expected_reg or value_recv != expected_value:
        return False
    
    # 校验CRC
    payload = resp[:-2]
    crc_recv = resp[-2] | (resp[-1] << 8)
    crc_calc = modbus_crc16(payload)
    if crc_recv != crc_calc:
        stats_rs485_crc_errors += 1
        return False
    
    return True

def poll_fan_status():
    """轮询风机状态，检测状态变化并同步上报"""
    global fan_onoff, fan_speed, fan_status, timer_rs485_poll
    
    req = build_modbus_read(MODBUS_DEV_ADDR, REG_STATUS_QUERY_START, REG_STATUS_QUERY_COUNT)
    rs485_write(req)
    resp = rs485_read(CONFIG_RS485_TIMEOUT_MS)
    
    data_bytes = parse_modbus_read_response(resp, MODBUS_DEV_ADDR, MODBUS_READ_FUNC, REG_STATUS_QUERY_COUNT)
    
    if data_bytes is None:
        prev_status = fan_status
        fan_status = False
        if prev_status != fan_status:
            log_warn("风机通信异常", ERR_RS485_TIMEOUT)
            send_data_response()
        timer_rs485_poll = time.ticks_ms()
        return
    
    # 解析寄存器数据（大端序）
    reg_power = (data_bytes[0] << 8) | data_bytes[1]
    # reg_mid = (data_bytes[2] << 8) | data_bytes[3]  # 保留寄存器
    reg_speed = (data_bytes[4] << 8) | data_bytes[5]
    
    # 保存旧状态用于比较
    prev_onoff = fan_onoff
    prev_speed = fan_speed
    prev_status = fan_status
    
    # 更新开关状态
    fan_onoff = (reg_power == 1)
    
    # 更新速度档位（风机关闭时速度为0，开启时速度应在1-4范围内）
    if fan_onoff:
        # 风机开启时，档位范围保护
        if reg_speed < 1:
            reg_speed = 1
        elif reg_speed > 4:
            reg_speed = 4
        fan_speed = reg_speed
    else:
        # 风机关闭时，速度寄存器为0，保持之前的速度值（用于下次开启时使用）
        # 如果之前没有速度值，默认设为1档
        if fan_speed == 0:
            fan_speed = 1
    
    fan_status = True
    
    # 检测状态变化
    state_changed = False
    
    # 检查通信状态变化
    if prev_status != fan_status:
        log_info("风机通信恢复")
        state_changed = True
    
    # 检查开关状态变化
    if prev_onoff != fan_onoff:
        log_info(f"风机状态变化: {'开启' if fan_onoff else '关闭'}")
        state_changed = True
    
    # 检查速度档位变化（仅在风机开启时检查，因为关闭时速度寄存器为0）
    if fan_onoff and prev_speed != fan_speed:
        log_info(f"风机速度变化: {prev_speed}档 -> {fan_speed}档")
        state_changed = True
    
    # 如果状态发生变化，上报到涂鸦平台
    if state_changed:
        send_data_response()
    
    timer_rs485_poll = time.ticks_ms()

def send_fan_power_to_rs485(on, retry_count=CONFIG_RS485_RETRY_COUNT):
    """发送风机开关命令，带重试和确认"""
    value = 1 if on else 0
    
    for attempt in range(retry_count):
        frame = build_modbus_write_single(MODBUS_DEV_ADDR, REG_POWER, value)
        rs485_write(frame)
        
        # 读取响应确认
        resp = rs485_read(CONFIG_RS485_TIMEOUT_MS)
        if verify_modbus_write_response(resp, MODBUS_DEV_ADDR, REG_POWER, value):
            log_info(f"风机{'开启' if on else '关闭'}成功")
            return True
        
        if attempt < retry_count - 1:
            time.sleep_ms(CONFIG_RS485_RETRY_DELAY_MS)
    
    log_error(f"风机{'开启' if on else '关闭'}失败", ERR_RS485_WRITE_FAIL)
    return False

def send_fan_speed_to_rs485(speed_level, retry_count=CONFIG_RS485_RETRY_COUNT):
    """发送风机速度命令，带重试和确认"""
    if speed_level < 1:
        speed_level = 1
    elif speed_level > 4:
        speed_level = 4
    
    for attempt in range(retry_count):
        frame = build_modbus_write_single(MODBUS_DEV_ADDR, REG_SPEED, speed_level)
        rs485_write(frame)
        
        # 读取响应确认
        resp = rs485_read(CONFIG_RS485_TIMEOUT_MS)
        if verify_modbus_write_response(resp, MODBUS_DEV_ADDR, REG_SPEED, speed_level):
            log_info(f"风机速度{speed_level}档设置成功")
            return True
        
        if attempt < retry_count - 1:
            time.sleep_ms(CONFIG_RS485_RETRY_DELAY_MS)
    
    log_error(f"风机速度{speed_level}档设置失败", ERR_RS485_WRITE_FAIL)
    return False

##############################################################
### 涂鸦数据包实现部分
##############################################################
def build_tuya_frame(command, data):
    """构建涂鸦协议帧"""
    frame = bytearray([0x55, 0xAA])  # 帧头
    frame.append(0x03)  # 协议版本
    frame.append(command)  # 命令字
    
    if data:
        # 添加数据
        data_bytes = bytes(data)
        data_len = struct.pack('>H', len(data_bytes))
        
        frame.extend(data_len)
        frame.extend(data_bytes)
    else:
        frame.extend(b'\x00')
        frame.extend(b'\x00')
        
    # 计算校验和
    checksum = sum(frame) & 0xFF
    frame.append(checksum)
    
    # 打印发送的帧
    log_debug(">> TX: " + bytes_to_hex(frame))
    return frame

##########################################################
### 不同命令字实现
##########################################################
def send_heartbeat_response():
    global first_heartbeat
    """发送心跳响应"""
    if first_heartbeat:
        heartbeat_resp = build_tuya_frame(0x00, b'\x00')  # 首次心跳响应命令
    else:
        heartbeat_resp = build_tuya_frame(0x00, b'\x01')  # 之后心跳响应命令
        
    uart.write(heartbeat_resp)
    first_heartbeat = False

def send_product_response(PID):
    """发送产品信息"""
    product_info = "{\"p\":\""
    product_info += PID
    product_info += "\",\"v\":\"1.0.0\",\"m\":0,\"low\":1}"
    
    log_debug(product_info)
    
    product_resp = build_tuya_frame(0x01, bytes(product_info, 'utf-8'))
    uart.write(product_resp)

def send_mode_response():
    """发送工作模式"""
    mode_resp = build_tuya_frame(0x02, None)
    uart.write(mode_resp)

def send_data_response():
    """发送数据上报"""
    global last_humidity, last_temp, fan_onoff, fan_speed, fan_status, timer_report
    
    #风机状态，DP ID： 1， bool
    fan_data = bytearray([0x01, 0x01, 0x00, 0x01])
    if fan_onoff:
        fan_data.append(0x01)
    else:
        fan_data.append(0x00)
    
    #风速，    DP ID： 3， enum
    speed_data = bytearray([0x03, 0x04, 0x00, 0x01])
    speed_data.append(fan_speed - 1)
    
    #设备状态，DP ID： 101, enum
    status_data = bytearray([0x65, 0x04, 0x00, 0x01])
    if fan_status:
        status_data.append(0x00)
    else:
        status_data.append(0x01)
    
    ###########################################
    ###   获取环境传感器读数
    ###   温度 DP ID 102
    ###   湿度 DP ID 103
    
    temp_data = bytearray([0x66, 0x02, 0x00, 0x04])
    temp_data.extend(int(last_temp).to_bytes(4, 'big'))
    
    hum_data = bytearray([0x67, 0x02, 0x00, 0x04])
    hum_data.extend(int(last_humidity).to_bytes(4, 'big'))
    
    # 组合数据并发送
    report_frame = build_tuya_frame(0x07, fan_data + speed_data + status_data + temp_data + hum_data)
    uart.write(report_frame)
    
    timer_report = time.ticks_ms()
    log_debug("数据上报完成")

def run_command(data):
    """处理涂鸦平台下发的命令 - 改进版：先执行RS485，确认成功后再更新状态"""
    global fan_onoff, fan_speed, pending_speed
    
    cmd = data[:2]
    
    if cmd == bytearray([0x01, 0x01]):
        # 开关命令
        status = data[4]
        target_onoff = (status == 1)
        
        # 先执行RS485命令
        if send_fan_power_to_rs485(target_onoff):
            # 命令成功，更新本地状态
            fan_onoff = target_onoff
            if target_onoff:
                # 开启时，如果有待机速度，应用它
                if pending_speed is not None:
                    if send_fan_speed_to_rs485(pending_speed):
                        fan_speed = pending_speed
                    pending_speed = None
                else:
                    # 默认1档
                    if send_fan_speed_to_rs485(1):
                        fan_speed = 1
            else:
                log_info("关闭风机")
        else:
            log_error("开关命令执行失败", ERR_RS485_WRITE_FAIL)
    
    elif cmd == bytearray([0x03, 0x04]):
        # 速度命令
        speed = data[4] + 1  # 转换为1~4档
        
        if speed < 1 or speed > 4:
            log_warn(f"无效速度档位: {speed}")
            send_data_response()
            return
        
        if fan_onoff:
            # 风机开启状态，直接设置速度
            if send_fan_speed_to_rs485(speed):
                fan_speed = speed
                log_info(f"设定风速{speed}档")
            else:
                log_error("速度命令执行失败", ERR_RS485_WRITE_FAIL)
        else:
            # 风机关闭状态，缓存速度设置
            pending_speed = speed
            log_info(f"风机关闭，速度{speed}档已缓存，开启时自动应用")
    
    # 上报当前状态
    send_data_response()

def update_network_status(data):
    """更新联网状态"""
    global current_state
    
    status = data[:1]

    if status == b'\x04':
        current_state = STATE_CLOUD_CONNECTED
        log_info("已连接到涂鸦云端")
        
    elif status == b'\x02' or status == b'\x03':
        current_state = STATE_PROVISIONED
        log_info("已配置网络，等待连接到涂鸦云端")
        
    else:
        current_state = STATE_UNPROVISIONED
        log_info("配网模式，等待配置")

def query_product_info():
    """查询产品信息"""
    log_debug("查询产品信息...")
    query_frame = build_tuya_frame(0x0A, b'')  # 产品信息查询命令
    uart.write(query_frame)

def query_network_status():
    """查询网络状态"""
    global timer_network_query
    
    log_debug("查询网络状态...")
    query_frame = build_tuya_frame(0x2B, b'')  # 网络状态查询命令
    uart.write(query_frame)
    timer_network_query = time.ticks_ms()

def process_rx_frame(frame):
    """处理接收到的数据帧"""
    global first_heartbeat, TUYA_PID
    
    # 打印接收到的原始帧
    log_debug("模组 RX: " + bytes_to_hex(frame))
    
    # 验证帧长度和校验和
    if len(frame) < 6 or sum(frame[:-1]) & 0xFF != frame[-1]:
        log_warn("无效帧或校验失败", ERR_TUYA_FRAME)
        return
    
    # 解析帧头
    protocol_ver = frame[2]
    command = frame[3]
    data_len = frame[4] | (frame[5] << 8)
    
    # 提取数据部分
    data = frame[6:6+data_len] if data_len > 0 else b''
    
    log_debug(f"收到命令: 0x{command:02X}, 数据长度: {data_len}")
    
    # 命令处理
    if command == 0x00:  # 心跳包
        send_heartbeat_response()
        log_debug("响应心跳")
    
    elif command == 0x01:  # 查询产品信息
        send_product_response(TUYA_PID)
        log_debug("返回产品信息")
    
    elif command == 0x02:  # 查询工作模式
        send_mode_response()
        log_debug("MCU与模块配合处理网络模式")
    
    elif command == 0x03 or command == 0x2B:  # 报告WIFI状态
        update_network_status(data)
        log_debug("模组报告联网状态")
    
    elif command == 0x04:  # 重置WIFI配置
        log_info("配网重置")
    
    elif command == 0x06:  # 命令下发
        run_command(data)
        log_debug("云端命令下发")
    
    elif command == 0x08:  # 状态查询，查询DP数据
        send_data_response()
        log_debug("状态查询")
    
    elif command == 0x23:  # 数据上报反馈
        log_debug("上报状态反馈")
    
    elif command == 0x0A:  # 产品信息响应
        if data_len > 0:
            # 解析产品信息
            pid = data[:16].decode('utf-8').strip('\x00')
            version = data[16:32].decode('utf-8').strip('\x00')
            log_info(f"产品信息 - PID: {pid}, 版本: {version}")
            product_info_queried = True
    
    elif command == 0x0B:  # 网络状态查询响应
        if data_len > 0 and data[0] in (0x01, 0x02, 0x03):
            status = data[0]
            if status == 0x01:
                current_state = STATE_UNPROVISIONED
                log_info("查询状态: 未配网")
            elif status == 0x02:
                current_state = STATE_PROVISIONED
                log_info("查询状态: 已配网未连云端")
            elif status == 0x03:
                current_state = STATE_CLOUD_CONNECTED
                log_info("查询状态: 云端已连接")
                # 连接云端后查询产品信息
                if not product_info_queried:
                    query_product_info()
    
    elif command == 0x0E:  # 配网响应
        if data and data[0] == 0x00:
            current_state = STATE_PROVISIONED
            log_info("配网成功")

def parse_uart_data():
    """解析UART接收缓冲区中的数据"""
    while uart.any():
        data = uart.read()
        process_rx_frame(data)

def reset_btn():
    """配网按钮处理 - 改进版：按键消抖和长按检测"""
    global current_state, timer_button, timer_button_press_start
    
    current_time = time.ticks_ms()
    
    # 消抖处理：每50ms检测一次
    if time.ticks_diff(current_time, timer_button) < CONFIG_BUTTON_DEBOUNCE_MS:
        return
    
    timer_button = current_time
    button_status = reset.value()
    
    if button_status == 0:  # 按下
        if timer_button_press_start == 0:
            # 记录按下开始时间
            timer_button_press_start = current_time
        else:
            # 检查是否达到长按时间
            press_duration = time.ticks_diff(current_time, timer_button_press_start)
            if press_duration >= CONFIG_BUTTON_LONG_PRESS_MS:
                # 触发配网
                current_state = STATE_UNPROVISIONED
                rest_wifi = build_tuya_frame(0x04, None)
                uart.write(rest_wifi)
                log_info("启动配网模式")
                timer_button_press_start = 0  # 重置
    else:  # 释放
        timer_button_press_start = 0  # 重置按下时间

def update_led():
    """根据网络状态更新LED指示灯（Pin 25）"""
    global led_state, timer_led
    
    current_time = time.ticks_ms()
    
    if current_state == STATE_UNPROVISIONED:  # 配网模式 - 快闪
        if time.ticks_diff(current_time, timer_led) >= 200:  # 200ms周期 (快闪)
            led_state = not led_state
            led.value(led_state)
            timer_led = current_time
    
    elif current_state == STATE_PROVISIONED:  # 已配网未连云端 - 慢闪
        if time.ticks_diff(current_time, timer_led) >= 1000:  # 1000ms周期 (慢闪)
            led_state = not led_state
            led.value(led_state)
            timer_led = current_time
    
    elif current_state == STATE_CLOUD_CONNECTED:  # 云端已连接 - 常亮
        led.value(1)  # 常亮

# 主程序初始化
current_state = STATE_UNPROVISIONED
led.value(0)  # 初始关闭LED

# 初始化所有节拍器
current_time = time.ticks_ms()
timer_report = current_time
timer_sensor = current_time
timer_rs485_poll = current_time
timer_network_query = current_time
timer_led = current_time
timer_button = current_time
timer_button_press_start = 0
timer_rgb_blink = current_time
rgb_blink_state = False

# 初始化传感器
try:
    i2c.writeto(AHT20_ADDR, bytes([0xBE, 0x08, 0x00]))
except:
    log_warn("AHT20初始化失败", ERR_SENSOR_READ)

log_info("系统启动...")
log_info("等待模块初始化...")

# 主循环
while True:
    parse_uart_data()  # 处理串口数据
    
    current_time = time.ticks_ms()
    
    # 数据上报：连到涂鸦平台后每30秒上报一次
    if (current_state == STATE_CLOUD_CONNECTED and 
        time.ticks_diff(current_time, timer_report) >= CONFIG_REPORT_INTERVAL_MS):
        send_data_response()
    
    # 传感器采样：每5秒采样一次
    if time.ticks_diff(current_time, timer_sensor) >= CONFIG_SENSOR_SAMPLE_INTERVAL_MS:
        update_env_data()
        aht20_trigger_measurement()
    
    # 网络状态查询：每5秒查询一次
    if time.ticks_diff(current_time, timer_network_query) >= CONFIG_NETWORK_QUERY_INTERVAL_MS:
        query_network_status()
    
    # RS485状态轮询：每2秒轮询一次
    if time.ticks_diff(current_time, timer_rs485_poll) >= CONFIG_RS485_POLL_INTERVAL_MS:
        poll_fan_status()
    
    # LED更新
    update_led()
    
    # RGB灯更新
    set_rgb(fan_speed)
    
    # 配网按钮检测
    reset_btn()
    
    time.sleep_ms(CONFIG_MAIN_LOOP_INTERVAL_MS)
