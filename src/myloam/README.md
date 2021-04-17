# 1. SLAM简介

常见的几个机器人研究的问题： 建图(Mapping)、定位(Localization)和路径规划（Path Planning）

同步定位与建图（SLAM）问题位于定位和建图的交集部分

<div align="center">
<img src="img/slam.png">
</div>

机器人在未知的环境中逐步建立起地图，然后根据地区确定自身位置，从而进一步定位。