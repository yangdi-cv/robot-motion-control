# 数据来源

TUM VI Benchmark，Schubert等，2018，CC BY4.0。
https://cvg.cit.tum.de/data/datasets/visual-inertial-dataset

本目录包含官方静止NPY的前120001行数值子集、room1的原始imu0/data.csv，以及用于本实验的两个30秒CSV。静止NPY容器重新保存，数值没有改写；30秒CSV重新排列为time_s、ax、ay、az、gx、gy、gz，并将运动原始纳秒时间换为相对秒。未经原作者或本实验声称的自有设备采集。

完整远端文件MD5未验证，本地内容哈希与已请求的HTTP范围见PROVENANCE.json、motion_download_ranges.json。
