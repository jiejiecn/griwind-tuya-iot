# griwind-tuya-iot
格兰斯柯新风系统接入涂鸦智能控制器和软件代码

本人小区开发商交付新风系统不支持米家、涂鸦等智能物联网平台，其自带的手机控制电路部分被阉割。新风厂家也不提供任何相关技术支持。</br>

通过研究其他暖通网关的资料，结合智能家居论坛中其他网友的经验，反复实验后找到了通过外挂系统（暖通网关的控制原理）对新风机的方法和通信指令，并将其与涂鸦智能平台对接，实现可以在涂鸦智能中通过手机远程获取、控制新风机状态。</br>

</br>
</br>

### 声明
本项目属于没有任何厂商资料的情况下的逆向工程，不对其正确性、稳定性提供任何保证。</br>

任何对原有新风机、控制面板的改动都可能影响质保服务，使用者需要自行承担其中的风险。</br>


</br>
</br>


### 限制
目前可以获取的信息包括以下两项：</br>
1. 新风工作状态： 启动 / 停止
2. 新风机风速： 1~4档，数字越大风速越高

目前无法获取的信息包括但不限于以下内容：</br>
1. 控制面板上显示温度
2. 新风、排风风阀状态
3. 滤网寿命

</br>
</br>

### 项目更新
#### 2025.12.29
已经可以实现App和面板（线控器）控制之间双向同步风机状态，具备实际使用条件。

备注：有新信号的RJ45接口线控器可以买到，但是新款阉割了485接口，无法通过面板侧连接485读取和控制，需要直连风机主板上485接口。



#### 2025.06.05
本项目专门适配格兰斯柯JW-150/250/350双向流新风系统，通过新风面板预留的485接口通信获取新风机工作状态。</br>

Panel: &emsp;  原厂面板资料</br>
PCB: &emsp;  外挂控制器PCB设计资料</br>
MCU: &emsp;  外挂控制器代码</br>



### 效果展示

外挂控制集成了AHT20温湿度传感器，用于获取室内的环境参数。
可以通过涂鸦云端下发指令操作新风机的开关、风速状态。

<img src="tuya_app1.jpg" width=40%> &emsp; <img src="tuya_app2.jpg" width=40%>


</br>
</br>


### PCB设计
MCU控制采用树莓派PICO，兼容RP2040 Zero或者RP2040 Tiny
RP2040 Zero集成了WS2812 RGB LED，可以用不同颜色组合表示风速状态。</br>

无线模块采用涂鸦WBR1，一款WIFI+蓝牙物联网模组。模组内置固件实现了与涂鸦云平台的通信协议，MCU只需要发送环境参数，获取云端指令并执行即可。MCU与无线模块的通信采用涂鸦WIFI标准串口通信协议，具体参见：XXXXXXXXX </br>

环境传感器采用AHT20温湿度传感器，采用I2C通信协议。</br>

控制器与新风面板连接，采用RS485通信接口，需要一个TTL-485模组，位于控制器PCB背面，负责将MCU指定发送到新风面板。</br>

控制器预留一颗微动开关，用于WIFI模组配网和设备绑定。</br>


<img src="pcb_top.jpg" width=50%>

<img src="pcb_bottom.jpg" width=50%>

</br>
</br>

### 参考资料
涂鸦WBR1模组： &emsp; https://developer.tuya.com/cn/docs/iot/wbr1-module-datasheet?id=K9duisiao4qpa

涂鸦串口协议： &emsp; https://developer.tuya.com/cn/docs/iot/tuya-cloud-universal-serial-port-access-protocol?id=K9hhi0xxtn9cb#title-0-%E4%B8%B2%E5%8F%A3%E9%80%9A%E4%BF%A1%E7%BA%A6%E5%AE%9A

AHT20模组: &emsp; https://files.seeedstudio.com/wiki/Grove-AHT20_I2C_Industrial_Grade_Temperature_and_Humidity_Sensor/AHT20-datasheet-2020-4-16.pdf

WS2812 RGB：&emsp; https://cdn-shop.adafruit.com/datasheets/WS2812.pdf







