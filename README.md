# 介绍 
开发工具VSCode  ESP-IDF，编译环境为esp-idf v5.4(https://docs.espressif.com/projects/esp-idf/zh_CN/v5.4/esp32/index.html)

复刻项目：[#第八届立创电赛#四轴飞行器Liguanxi-UAV（空心杯无人机飞控）](https://oshwhub.com/liguanxi/si-zhou-fei-xing-qi-ESP-Liguanxi)，原编译环境为esp-idf 4.4

## 如何使用
在VSCODE安装ESP-IDF，并安装ESP-IDF v5.4.0

## 文件内容

项目 **ESP32_UAV** 包含一个用 C 语言编写的源文件 main.c，该文件位于 main 文件夹中。

ESP-IDF 项目使用 CMake 进行构建。项目构建配置包含在 CMakeLists.txt 文件中，这些文件提供了一组指令和说明，描述了项目的源文件和目标（可执行文件、库或两者皆有）。

以下是项目文件夹中其余文件的简短说明：

```
├── CMakeLists.txt
├── main
│   ├── CMakeLists.txt
│   └── main.c
└── README.md                  这是您当前正在阅读的文件
```
