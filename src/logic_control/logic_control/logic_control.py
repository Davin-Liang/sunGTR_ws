#!/usr/bin/env python3
import rclpy
from .sust_gtr import SUST_GTR

def main(args=None):
    rclpy.init(args=args)
    Logician = SUST_GTR("SUST_GTR")
    
    # --------------------------- 任务一 ---------------------------
    # 发布开始寻线的指令
    Logician.order_car_to_start()
    # 等待寻线完成
    Logician.wait_patrol_finish()
    # 识别二维码
    Logician.wait_detect_QR_finish()
    # 将识别结果上传到数字环境
    Logician.upload_QR_messge()    
    # ------------------------- 任务一结束 -------------------------

    # --------------------------- 任务二 ---------------------------
    # 根据任务名称进行遥控操作
    Logician.get_logger().info("请按 [r] 进入键盘控制模式")
    Logician.wait_exit_keyboard_mode()
    # ------------------------- 任务二结束 -------------------------
    
    # --------------------------- 任务三 ---------------------------
    # 退出键盘控制模式，并发布开始寻线的指令
    Logician.order_car_to_start()
    # 等待寻线完成
    Logician.wait_patrol_finish()

    rclpy.spin(Logician) # 保持节点运行，检测是否收到退出指令（Ctrl+C）
    rclpy.shutdown() # 关闭rclpy
