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
## 相关资料
- [尚硅谷嵌入式项目平衡车教程， STM32项目实战之两轮平衡车](https://www.bilibili.com/video/BV12W421X7Ka?p=22)
- [SPL06电容式压力传感器数据读取与处理(基于STM32)](https://blog.csdn.net/qq_40598185/article/details/119347845?ops_request_misc=%257B%2522request%255Fid%2522%253A%2522b1b74e98a2848eda046f0182781c7856%2522%252C%2522scm%2522%253A%252220140713.130102334..%2522%257D&request_id=b1b74e98a2848eda046f0182781c7856&biz_id=0&utm_medium=distribute.pc_search_result.none-task-blog-2~all~top_positive~default-1-119347845-null-null.142%5Ev100%5Epc_search_result_base1&utm_term=spl06&spm=1018.2226.3001.4187)
- [SPL06-01手册](https://item.szlcsc.com/2782076.html?lcsc_vid=RVRYV1ECFFINBVVQFgMNBABSQVALUFIFEVYKU1ZREwQxVlNSR1RZXldTTlJeVjsOAxUeFF5JWBYZEEoBGA4JCwFIFA4DSA%3D%3D)