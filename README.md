# 单片机原理及应用课程设计

这是我的单片机课设项目，主要功能是通过 MQTT 向 ESP-32 发送消息控制一个 8x32 的 RGB LED 阵列的显示。

由于课程受到了疫情影响停了不少时间，因此成品和早起设计有很大的差别。

硬件：

* NodeMCU ESP32-S (EPS32_REV0)
* Ws2812B RGB LED Matrix (8x32)
* 5V 供电，杜邦线等

软件开发环境：

* JetBrains CLion 2021.3.x
* PlatformIO for CLion

除此之外，你还需要：

* 一个 Wifi 接入点
* 一个 MCU 可访问的 MQTT Broker
* 一个用于发布 MQTT 消息的 MQTT Client

# 许可证

本项目使用 [GNU General Public License v3.0](./LICENSE) 许可证
