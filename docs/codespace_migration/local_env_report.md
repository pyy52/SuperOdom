# Local Environment Report

## Date
2026年 09月 21日 星期一 20:55:02 CST

## OS
Distributor ID:	Ubuntu
Description:	Ubuntu 20.04.6 LTS
Release:	20.04
Codename:	focal

## Kernel
Linux pyy 5.15.0-139-generic #149~20.04.1-Ubuntu SMP Wed Apr 16 08:29:56 UTC 2025 x86_64 x86_64 x86_64 GNU/Linux

## CPU
架构：                                x86_64
CPU 运行模式：                        32-bit, 64-bit
字节序：                              Little Endian
Address sizes:                        48 bits physical, 48 bits virtual
CPU:                                  16
在线 CPU 列表：                       0-15
每个核的线程数：                      2
每个座的核数：                        8
座：                                  1
NUMA 节点：                           1
厂商 ID：                             AuthenticAMD
CPU 系列：                            25
型号：                                116
型号名称：                            AMD Ryzen 7 7840H w/ Radeon 780M Graphics
步进：                                1
Frequency boost:                      enabled
CPU MHz：                             3800.000
CPU 最大 MHz：                        6679.6870
CPU 最小 MHz：                        1600.0000
BogoMIPS：                            7586.19
虚拟化：                              AMD-V
L1d 缓存：                            256 KiB
L1i 缓存：                            256 KiB
L2 缓存：                             8 MiB
L3 缓存：                             16 MiB
NUMA 节点0 CPU：                      0-15
Vulnerability Gather data sampling:   Not affected
Vulnerability Itlb multihit:          Not affected
Vulnerability L1tf:                   Not affected
Vulnerability Mds:                    Not affected
Vulnerability Meltdown:               Not affected
Vulnerability Mmio stale data:        Not affected
Vulnerability Reg file data sampling: Not affected
Vulnerability Retbleed:               Not affected
Vulnerability Spec rstack overflow:   Mitigation; safe RET
Vulnerability Spec store bypass:      Mitigation; Speculative Store Bypass disabled via prctl and seccomp
Vulnerability Spectre v1:             Mitigation; usercopy/swapgs barriers and __user pointer sanitization
Vulnerability Spectre v2:             Mitigation; Enhanced / Automatic IBRS; IBPB conditional; STIBP always-on; RSB filling; PBRSB-eIBRS Not affected; BHI Not affected
Vulnerability Srbds:                  Not affected
Vulnerability Tsx async abort:        Not affected

## Memory
              总计         已用        空闲      共享    缓冲/缓存    可用
内存：        14Gi       6.2Gi       2.9Gi       268Mi       5.6Gi       8.0Gi
交换：       2.0Gi       1.3Gi       734Mi

## Disk
文件系统        容量  已用  可用 已用% 挂载点
udev            7.4G     0  7.4G    0% /dev
tmpfs           1.5G  2.5M  1.5G    1% /run
/dev/nvme1n1p2  938G  344G  547G   39% /
tmpfs           7.5G  293M  7.2G    4% /dev/shm
tmpfs           5.0M  4.0K  5.0M    1% /run/lock
tmpfs           7.5G     0  7.5G    0% /sys/fs/cgroup
/dev/loop0      128K  128K     0  100% /snap/bare/5
/dev/loop1       67M   67M     0  100% /snap/core24/1643
/dev/loop2      607M  607M     0  100% /snap/gnome-46-2404/153
/dev/loop3      615M  615M     0  100% /snap/gnome-46-2404/164
/dev/loop4      402M  402M     0  100% /snap/mesa-2404/1839
/dev/loop5       51M   51M     0  100% /snap/snapd/27738
/dev/loop6      1.2G  1.2G     0  100% /snap/libreoffice/374
/dev/loop7       51M   51M     0  100% /snap/snapd/27710
/dev/loop8       92M   92M     0  100% /snap/gtk-common-themes/1535
/dev/loop9      395M  395M     0  100% /snap/mesa-2404/1165
/dev/loop10     1.2G  1.2G     0  100% /snap/libreoffice/377
/dev/nvme0n1p1  256M   82M  175M   32% /boot/efi
tmpfs           1.5G   24K  1.5G    1% /run/user/126
tmpfs           1.5G  104K  1.5G    1% /run/user/1000

## ROS Environment
ROS_VERSION=1
ROS_PYTHON_VERSION=3
ROS_PACKAGE_PATH=/home/peter/catkin_ws/src:/opt/ros/noetic/share
ROS_ETC_DIR=/opt/ros/noetic/etc/ros
CMAKE_PREFIX_PATH=/home/peter/catkin_ws/devel:/opt/ros/noetic
PYTHONPATH=/home/peter/catkin_ws/devel/lib/python3/dist-packages:/opt/ros/noetic/lib/python3/dist-packages
ROS_MASTER_URI=http://localhost:11311
LD_LIBRARY_PATH=/home/peter/catkin_ws/devel/lib:/opt/ros/noetic/lib
ROS_ROOT=/opt/ros/noetic/share/ros
ROS_DISTRO=noetic

## ROS Version Files
总用量 12
drwxr-xr-x  3 root root 4096 11月  4  2025 .
drwxr-xr-x 14 root root 4096 9月  17 22:43 ..
drwxr-xr-x  7 root root 4096 11月  4  2025 noetic

## Compiler
gcc (Ubuntu 9.4.0-1ubuntu1~20.04.2) 9.4.0
Copyright (C) 2019 Free Software Foundation, Inc.
This is free software; see the source for copying conditions.  There is NO
warranty; not even for MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

g++ (Ubuntu 9.4.0-1ubuntu1~20.04.2) 9.4.0
Copyright (C) 2019 Free Software Foundation, Inc.
This is free software; see the source for copying conditions.  There is NO
warranty; not even for MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.

cmake version 3.16.3

CMake suite maintained and supported by Kitware (kitware.com/cmake).
GNU Make 4.2.1
为 x86_64-pc-linux-gnu 编译
Copyright (C) 1988-2016 Free Software Foundation, Inc.
许可证：GPLv3+：GNU 通用公共许可证第 3 版或更新版本<http://gnu.org/licenses/gpl.html>。
本软件是自由软件：您可以自由修改和重新发布它。
1.10.0

## Python
Python 3.8.10
pip 25.0.1 from /home/peter/.local/lib/python3.8/site-packages/pip (python 3.8)

## Git
git version 2.25.1
gh version 2.101.0 (2026-09-15)
https://github.com/cli/cli/releases/tag/v2.101.0

## CUDA / NVIDIA
Mon Sep 21 20:55:02 2026       
+---------------------------------------------------------------------------------------+
| NVIDIA-SMI 535.230.02             Driver Version: 535.230.02   CUDA Version: 12.2     |
|-----------------------------------------+----------------------+----------------------+
| GPU  Name                 Persistence-M | Bus-Id        Disp.A | Volatile Uncorr. ECC |
| Fan  Temp   Perf          Pwr:Usage/Cap |         Memory-Usage | GPU-Util  Compute M. |
|                                         |                      |               MIG M. |
|=========================================+======================+======================|
|   0  NVIDIA GeForce RTX 4060 ...    Off | 00000000:01:00.0  On |                  N/A |
| N/A   41C    P3               6W /  60W |    612MiB /  8188MiB |     21%      Default |
|                                         |                      |                  N/A |
+-----------------------------------------+----------------------+----------------------+
                                                                                         
+---------------------------------------------------------------------------------------+
| Processes:                                                                            |
|  GPU   GI   CI        PID   Type   Process name                            GPU Memory |
|        ID   ID                                                             Usage      |
|=======================================================================================|
|    0   N/A  N/A      1110      G   /usr/lib/xorg/Xorg                           53MiB |
|    0   N/A  N/A      1896      G   /usr/lib/xorg/Xorg                          190MiB |
|    0   N/A  N/A      2044      G   /usr/bin/gnome-shell                         46MiB |
|    0   N/A  N/A    946102      G   ...cess-track-uuid=3190708988185955192      181MiB |
|    0   N/A  N/A   1174492      G   ...cess-track-uuid=3190708988185955192      121MiB |
+---------------------------------------------------------------------------------------+
nvcc: NVIDIA (R) Cuda compiler driver
Copyright (c) 2005-2019 NVIDIA Corporation
Built on Sun_Jul_28_19:07:16_PDT_2019
Cuda compilation tools, release 10.1, V10.1.243
