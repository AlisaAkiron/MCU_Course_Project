# 单片机原理及应用课程设计

这是我的单片机课设项目，主要功能是通过 MQTT 向 ESP-32 发送消息控制一个 8x32 的 RGB LED 阵列的显示。

由于课程受到了疫情影响停了不少时间，因此成品和早起设计有很大的差别。

## 硬件

* NodeMCU ESP32-S (EPS32_REV0)
* Ws2812B RGB LED Matrix (8x32)
* 5V 供电，杜邦线等

## 软件开发环境

* JetBrains CLion 2021.3.x
* PlatformIO for CLion

## 除此之外，你还需要：

* 一个 Wifi 接入点
* 一个 MCU 可访问的 MQTT Broker
* 可以向 MQTT Broker 发送消息的客户端软件

## 客户端软件

在 [client](./client) 文件夹中是使用 .NET 6 编写的两个客户端软件。

* `ESPController` 为控制软件
* `ESPMonitor` 为监测 `esp/log` Topic 的日志监测软件

## 使用

1. 准备并启动一个 MQTT 服务器，推荐可以使用 Docker 部署 EMQX Broker

2. 重命名 [include/MqttConfiguration.h.example](./include/MqttConfiguration.h.example) 为 `MqttConfiguration.h` 并修改配置

3. 修改 [client/ESPController/Program.cs](./client/ESPController/Program.cs) 和 [client/ESPMonitor/Program.cs](./client/ESPMonitor/Program.cs) 中的配置

4. 连接 ESP32，打开 PlatformIO 项目，编译并烧写程序

5. 打开 `ESPMonitor` 监控从网络发送的日志

6. 将 WIFI 重制引脚（默认为 GPIO 16）接高电平 3.3V 

7. 将 GPIO 23 连接 WS2812B 的 DIN，5V 和 GND 接 WS2812B 的 5V 和 GND

8. 按下 ESP32 的使能/重制按钮

9. 使用手机或电脑等能够连接 WIFI 的设备连接 `NodeMCU` 开头的 WIFI

10. 打开浏览器登录 `192.168.4.1` 打开 ESP32 WIFI 配置页面，配置 ESP32 将要连接的 WIFI 热点

11. 打开 `ESPController` 发送指令

## ESPController

一个简单的指令发送程序

* `mode single [X] [Y] [R] [G] [B]` 将 (X, Y) 点的像素设置为对应 RGB 色彩

* `mode picture [PATH_TO_PICTURE(PNG,JPEG,BMP)]` 显示一个静态图像，图像必须是 32 * 8 尺寸

* `mode picture_group [PATH_TO_PICTURE_FOLDER] [TIMES]` 循环 TIMES 文件夹中的图片显示，图片必须是 32 * 8 的尺寸，TIMES 必须是大于 0 的正整数，设置为 `inf` 则将会无限循环

* `mode clock [GMT_TIME_ZONE]` 显示一个时钟并设置时区

* `mode stop` 停止当前显示并清空屏幕

* `command set general_lightness [LIGHTNESS(0-50)]` 设置亮度（由于 WS2812B 没有单独的白色灯珠，所以该设置项在显示非白色时有时候不一定管用）

* `command set digit_color [R] [G] [B]` 设置时钟模式的显示色彩

## 其他

[demo](./demo) 文件夹中有示例像素动画图片序列

# 许可证

本项目使用 [GNU General Public License v3.0](./LICENSE) 许可证
