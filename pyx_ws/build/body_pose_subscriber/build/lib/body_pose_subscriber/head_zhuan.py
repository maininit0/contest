#!/usr/bin/env python
#
# *********     Sync Write Example      *********
#
# 多舵机同步写入控制程序（改进版）
# 可以控制ID为1-9的舵机，并通过列表传入每个舵机的目标角度

import sys
import os
import time

from scservo_sdk import *
# 初始化端口处理器实例
portHandler = PortHandler('/dev/ttyACM0')      # 创建端口处理器对象

# 初始化数据包处理器实例
packetHandler = sms_sts(portHandler)           # 创建适用于SMS/STS系列舵机的数据包处理器

# 角度转换为位置值函数
def angle_to_position(angle):
    """
    将角度(0-360度)转换为舵机位置值(0-4095)
    """
    # 确保角度在0-360范围内
    angle = angle % 360
    # 线性映射: 0-360度 映射到 0-4095
    position = int((angle / 360) * 4095)
    return position

# 位置值转换为角度函数
def position_to_angle(position):
    """
    将舵机位置值(0-4095)转换为角度(0-360度)
    """
    # 确保位置在0-4095范围内
    position = max(0, min(position, 4095))
    # 线性映射: 0-4095 映射到 0-360度
    angle = (position / 4095) * 360
    return angle

# 同步控制多个舵机移动到指定角度
def sync_move_to_angles(angle_list, speed=2550, acceleration=255):  # 默认速度提高到120，加速度提高到80
    """
    同步控制多个舵机移动到指定角度
    
    参数:
    angle_list -- 包含9个舵机目标角度的列表(度)
    speed -- 运动速度参数，默认120 (约87.84rpm)
    acceleration -- 加速度参数，默认80 (约696deg/s²)
    
    返回:
    True成功，False失败
    """
    if len(angle_list) != 9:
        print("错误: 角度列表必须包含9个值，对应9个舵机")
        return False
    
    # 清除之前的参数
    packetHandler.groupSyncWrite.clearParam()
    
    # 计算最大位置差，用于确定等待时间
    max_position_diff = 0
    
    # 为每个舵机添加参数
    for i, angle in enumerate(angle_list):
        scs_id = i + 1  # 舵机ID从1开始
        position = angle_to_position(angle)  # 将角度转换为位置值
        
        # 获取当前位置（可选，用于计算运动时间）
        current_position = 0  # 默认值
        try:
            # 尝试读取当前位置
            scs_present_position, scs_comm_result, scs_error = packetHandler.ReadPos(scs_id)
            if scs_comm_result == COMM_SUCCESS and scs_error == 0:
                current_position = scs_present_position
        except:
            # 如果读取失败，使用默认值
            pass
        
        # 计算位置差的绝对值，并更新最大位置差
        position_diff = abs(position - current_position)
        max_position_diff = max(max_position_diff, position_diff)
        
        # 将参数添加到同步写入命令
        scs_addparam_result = packetHandler.SyncWritePosEx(scs_id, position, speed, acceleration)
        if scs_addparam_result != True:
            print(f"[ID:{scs_id:03d}] 添加参数失败")
            return False
    
    # 执行同步写入命令
    scs_comm_result = packetHandler.groupSyncWrite.txPacket()
    if scs_comm_result != COMM_SUCCESS:
        print("%s" % packetHandler.getTxRxResult(scs_comm_result))
        return False
    
    # 清除参数
    packetHandler.groupSyncWrite.clearParam()
    
    # 计算等待时间
    # 如果没有获取到当前位置，使用最大值4095作为保守估计
    if max_position_diff == 0:
        max_position_diff = 4095
    
    # 由于速度加快，等待时间可以适当缩短
    wait_time = ((max_position_diff)/(speed*50) + (speed*50)/(acceleration*100) + 0.05)
    time.sleep(wait_time)
    
    return True

# 主程序
def main():
    # 打开串口
    if portHandler.openPort():
        print("成功打开端口")
    else:
        print("打开端口失败")
        return
    
    # 设置端口波特率
    if portHandler.setBaudRate(1000000):
        print("成功设置波特率")
    else:
        print("设置波特率失败")
        portHandler.closePort()
        return
    
    try:
        while True:
            # 示例2: 所有舵机移动到不同角度
            print("所有舵机移动到不同角度...")
            angles = [50, 45, 90, 110, 261, 115, 190, 315, 360]   #   传入角度
            sync_move_to_angles(angles, speed=2550, acceleration=255)  # 使用更快的速度
            time.sleep(0.1)  # 等待时间缩短
            
            # 是否继续
            choice = input("是否继续演示? (y/n): ").lower()
            if choice != 'y':
                break
                
    except KeyboardInterrupt:
        print("程序被用户中断")
    finally:
        # 关闭串口
        portHandler.closePort()
        print("端口已关闭")

if __name__ == "__main__":
    main()