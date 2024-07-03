# python_scripts
this responsitory is created for python scripts of ros

## gps_node_tcp

本脚本运行环境ros1，文件夹中包含了脚本以及三个话题对饮的输出示例。

该脚本使用socket通信，并且遵循gps_rtk专用的通信协议NMEA，NMEA-0183是美国国家海洋电子协会为海用电子设备制定的标准格式。它包含了定位时间，纬度，经度，高度，定位所用的卫星数，DO[P值](https://so.csdn.net/so/search?q=P值&spm=1001.2101.3001.7020)，差分状态和校正时段等很多信息，具体的协议内容可以自行搜索。

为了解析NMEA格式的数据，本脚本调用了pynmea2库来解析，是需调用prase()函数就可解析socket端口传输过来的数据。

**关于方向角：**

两个gps天线从主天线到从天线为gps的方向，其与正北的夹角就为方向角，顺时针旋转方向角从0到360度逐渐增大。不同的安装角度以及不同的正方向定义都会存在不同的方向角变换，例如有的车型是以正东为零度，逆时针从0到360度，天线正方向和车正方向偏差45度左右，本脚本就是这个情况。

## jodell ros驱动包

为jodell公司的夹爪开发的ros驱动包
