import machine
import ubinascii
import ujson
import time
import math
import struct
import neopixel
from machine import ADC, Pin, UART, I2C

TUYA_PID = "b7zdedszp3gq3k9t" # 涂鸦产品ID

# 硬件初始化
uart = UART(0, baudrate=115200, tx=Pin(0), rx=Pin(1))  # UART0使用GP0和GP1，涂鸦模块通信端口
led = Pin(25, Pin.OUT)  # 模块25脚LED，网络状态LED
np = neopixel.NeoPixel(machine.Pin(16), 1) # GPIO 16连接WS2812，风速状态LED

# AHT20传感器
i2c = I2C(1, scl=Pin(7), sda=Pin(6), freq=100000)
AHT20_ADDR = 0x38

reset = Pin(28, Pin.IN, Pin.PULL_UP) # 模块28脚作为配网按钮
reset_timer = 0

# 状态常量
STATE_UNPROVISIONED = 0
STATE_PROVISIONED = 1
STATE_CLOUD_CONNECTED = 2

# 全局变量
current_state = STATE_UNPROVISIONED

last_report_time = 0

led_timer = 0
led_state = False

first_heartbeat = True       # 首次心跳请求

###########################################
###        环境和设备状态变量           #####
last_temp = 0                 #室内温度
last_humidity = 0             #室内湿度

fan_onoff = False              #风机状态，启动/关闭
fan_speed = 1                 #风速，1~4档
fan_status = True             #风机通信状态，是否响应指令

UPDATE_INTERVAL = 30000



#############################################################
### AHT20 温湿度获取部分
def aht20_trigger_measurement():
    # 触发一次测量
    i2c.writeto(AHT20_ADDR, bytes([0xAC, 0x33, 0x00]))
    time.sleep(0.08)  # 等待测量完成

def aht20_read_data():
    data = i2c.readfrom(AHT20_ADDR, 7)
    if (data[0] & 0x80) == 0:  # 检查状态位，确保数据有效
        raw_humi = ((data[1] << 12) | (data[2] << 4) | (data[3] >> 4)) & 0xFFFFF
        raw_temp = ((data[3] & 0x0F) << 16) | (data[4] << 8) | data[5]
        humidity = (raw_humi / 1048576.0) * 100
        temperature = (raw_temp / 1048576.0) * 200 - 50
        return temperature, humidity
    else:
        return None, None

def update_env_data():
    global last_temp, last_humidity
    
    last_temp, last_humidity = aht20_read_data()
    print("Temp & Humi: ", last_temp, last_humidity)
    
    


#########################################################
### RGB灯变化
def set_rgb(level):
    global np
    
    ##########################################
    ### 1档  蓝色
    ### 2档  绿色
    ### 3档  橙色
    ### 4档  红色 Max
    
    if fan_onoff:
        if level == 1:
            np.fill((0, 0, 16))
            np.write()
        
        if level == 2:
            np.fill((8, 32, 0))
            np.write()
        
        if level == 3:
            np.fill((32, 16, 0))
            np.write()
        
        if level == 4:
            np.fill((32, 0, 0))
            np.write()
    else:
        np.fill((0, 0, 0))
        np.write()
    

def bytes_to_hex(byte_data):
    """将字节数据转换为十六进制字符串"""
    return ''.join('{:02X} '.format(b) for b in byte_data)


##############################################################
### 涂鸦数据包实现部分
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
    print(f">> TX: {bytes_to_hex(frame)}")
    return frame


##########################################################
### 不同命令字实现
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
    product_info += "\",\"v\":\"1.0.0\",\"m\":0}"
    
    print(product_info)
    
    product_resp = build_tuya_frame(0x01, bytes(product_info, 'utf-8'))
    uart.write(product_resp)
    
    

def send_mode_response():
    global rest_wifi
    """发送工作模式"""
    mode_resp = build_tuya_frame(0x02, None)
    uart.write(mode_resp)


def send_data_response():
    global last_humidity, last_temp, fan_onoff, fan_speed, fan_status, last_report_time
    
    #风机状态，DP IP： 1， bool
    fan_data = bytearray([0x01, 0x01, 0x00, 0x01])
    if fan_onoff:
        fan_data.append(0x01)
    else:
        fan_data.append(0x00)
    
    #风速，    DP IP： 3， enum
    speed_data = bytearray([0x03, 0x04, 0x00, 0x01])
    speed_data.append(fan_speed - 1)
    
    #设备状态，DP IP： 101, enum
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
    
    last_report_time = time.ticks_ms()
    print("数据上报完成")

def run_command(data):
    global last_report_time, fan_onoff, fan_speed, np
    
    cmd = data[:2]
    
    if cmd == bytearray([0x01, 0x01]):
        status = data[4]
        if status == 0:
            fan_onoff = False
            print("关闭风机")
        
        elif status == 1:
            fan_onoff = True
            fan_speed = 1
            print("开启风机，风速1档")
    
    elif  cmd == bytearray([0x03, 0x04]):
        speed = data[4]
        fan_speed = speed + 1
        
        print("设定风速" + str(fan_speed) + "档")
        
    print("执行完成")
    
    send_data_response()
    

def update_network_status(data):
    """更新联网状态"""
    global  current_state
    
    status = data[:1]

    if status == b'\x04':
        current_state = STATE_CLOUD_CONNECTED
        print("已连接到涂鸦云端")
        
    elif  status == b'\x02' or status == b'\x03':
        current_state = STATE_PROVISIONED
        print("已配置网络，等待连接到涂鸦云端")
        
    else:
        current_state = STATE_UNPROVISIONED
        print("配网模式，等待配置")
    
    

def query_product_info():
    """查询产品信息"""
    print("查询产品信息...")
    query_frame = build_tuya_frame(0x0A, b'')  # 产品信息查询命令
    uart.write(query_frame)

def query_network_status():
    """查询网络状态"""
    global  last_report_time
    
    print("查询网络状态...")
    query_frame = build_tuya_frame(0x2B, b'')  # 网络状态查询命令
    uart.write(query_frame)
    
    last_report_time = time.ticks_ms()

def process_rx_frame(frame):
    """处理接收到的数据帧"""
    global first_heartbeat, TUYA_PID
    
    # 打印接收到的原始帧
    print(f"模组 RX: {bytes_to_hex(frame)}")
    
    # 验证帧长度和校验和
    if len(frame) < 6 or sum(frame[:-1]) & 0xFF != frame[-1]:
        print("无效帧或校验失败")
        return
    
    # 解析帧头
    protocol_ver = frame[2]
    command = frame[3]
    data_len = frame[4] | (frame[5] << 8)
    
    # 提取数据部分
    data = frame[6:6+data_len] if data_len > 0 else b''
    
    print(f"收到命令: 0x{command:02X}, 数据长度: {data_len}")
    
    # 命令处理
    if command == 0x00:  # 心跳包
        send_heartbeat_response()
        print("响应心跳")
    
    elif command == 0x01:  # 查询产品信息
        send_product_response(TUYA_PID)
        print("返回产品信息")
    
    elif command == 0x02:  # 查询工作模式
        send_mode_response()
        print("MCU与模块配合处理网络模式")
    
    elif command == 0x03 or command == 0x2B:  # 报告WIFI状态
        update_network_status(data)
        print("模组报告联网状态")
    
    elif command == 0x04:  # 重置WIFI配置
        print("配网重置")
    
    elif command == 0x06:  # 命令下发
        run_command(data)
        print("云端命令下发")
    
    elif command == 0x08:  # 状态查询，查询DP数据
        send_data_response()
        print("状态查询")
    
    elif command == 0x23:  # 数据上报反馈
        print("上报状态反馈")
    
    elif command == 0x06:  # 网络状态主动上报
        if data and data[0] == 0x03:  # 云端连接成功
            current_state = STATE_CLOUD_CONNECTED
            print("云端已连接")
            # 连接云端后查询产品信息
            if not product_info_queried:
                query_product_info()
        elif data and data[0] == 0x02:  # 已配网未连云端
            current_state = STATE_PROVISIONED
            print("已配网未连云端")
        else:  # 未配网
            current_state = STATE_UNPROVISIONED
            print("未配网")
    
    elif command == 0x0A:  # 产品信息响应
        if data_len > 0:
            # 解析产品信息
            pid = data[:16].decode('utf-8').strip('\x00')
            version = data[16:32].decode('utf-8').strip('\x00')
            print(f"产品信息 - PID: {pid}, 版本: {version}")
            product_info_queried = True
    
    elif command == 0x0B:  # 网络状态查询响应
        if data_len > 0 and data[0] in (0x01, 0x02, 0x03):
            status = data[0]
            if status == 0x01:
                current_state = STATE_UNPROVISIONED
                print("查询状态: 未配网")
            elif status == 0x02:
                current_state = STATE_PROVISIONED
                print("查询状态: 已配网未连云端")
            elif status == 0x03:
                current_state = STATE_CLOUD_CONNECTED
                print("查询状态: 云端已连接")
                # 连接云端后查询产品信息
                if not product_info_queried:
                    query_product_info()
    
    elif command == 0x0E:  # 配网响应
        if data and data[0] == 0x00:
            current_state = STATE_PROVISIONED
            print("配网成功")

def parse_uart_data():
    """解析UART接收缓冲区中的数据"""
    while uart.any():
        data = uart.read()
        process_rx_frame(data)


def reset_btn():
    ### 配网按钮，长按5秒触发重新配网 ###
    ### 配网时红色LED快速闪烁 ###
    global reset_timer, current_state
    
    status = reset.value()
    
    if status == 0:
        reset_timer += 1
    else:
        reset_timer = 0
    
    if reset_timer >= 100:
        current_state = STATE_UNPROVISIONED
        # 发送进入配网模式命令
        rest_wifi = build_tuya_frame(0x04, None)
        uart.write(rest_wifi)
        print("启动配网模式")    
    
        reset_timer = 0


def update_led():
    """根据网络状态更新LED指示灯"""
    global led_timer, led_state
    
    current_time = time.ticks_ms()
    
    if current_state == STATE_UNPROVISIONED:  # 配网模式
        if current_time - led_timer >= 200:  # 200ms周期 (1秒2.5次)
            led_state = not led_state
            led.value(led_state)
            led_timer = current_time
    
    elif current_state == STATE_PROVISIONED:  # 已配网未连云端
        if current_time - led_timer >= 1000:  # 1000ms周期 (2秒1次)
            led_state = not led_state
            led.value(led_state)
            led_timer = current_time
    
    elif current_state == STATE_CLOUD_CONNECTED:  # 云端已连接
        led.value(1)  # 常亮

# 主程序初始化
led.value(0)  # 初始关闭LED
startup_timer = time.ticks_ms()
# 初始化传感器
i2c.writeto(AHT20_ADDR, bytes([0xBE, 0x08, 0x00]))

print("系统启动...")
print("等待模块初始化...")

# 主循环
while True:
    parse_uart_data()  # 处理串口数据
    
    current_time = time.ticks_ms()
    
    # 连到涂鸦平台后后每30秒上报一次数据
    if current_time - last_report_time >= UPDATE_INTERVAL and current_state == STATE_CLOUD_CONNECTED:
        send_data_response()
    
    if current_time - last_report_time >= 5000:
        update_env_data()
        
        query_network_status()
        
        aht20_trigger_measurement()
    
    
    update_led()         # 更新LED状态
    set_rgb(fan_speed)   # 更新RGB灯状态
    reset_btn()          # 配网按钮状态
    
    time.sleep_ms(50)  
