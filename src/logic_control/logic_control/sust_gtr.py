import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String
import cv2
from sensor_msgs.msg import Image, Imu
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
# from pyzbar.pyzbar import decode
# from origincar_msg.msg import Data
from ai_msgs.msg import PerceptionTargets
import time
import math

class SUST_GTR(Node):
    def __init__(self, name):
        super().__init__(name)
        self.get_logger().info("大家好，我是%s!" % name)
        self.action_command = Bool()
        self.QR_message = String()
        self.bridge = CvBridge()
        self.patrol_result = False
        self.detect_QR = True
        self.keyboard_mode = False

        self.patrol_result_subscribe_   = self.create_subscription(Bool,    "/patrol_result", self.patrol_result_callback, 10)
        self.image_subscribe_           = self.create_subscription(Image,           "/image", self.image_callback, 10)
        # self.robotpose_subscribe_       = self.create_subscription(Data,        "/robotpose", self.robotpose_callback, 10)
        self.imu_subscribe_             = self.create_subscription(Imu,          "/imu/data", self.imu_callback, 10)
        self.patrol_command_publisher   = self.create_publisher(Bool, "patrol_command", 10)
        self.QR_message_publisher       = self.create_publisher(String, "QR_message", 10)

        self.action_command.data = True

    def imu_callback(self):
        pass

    def robotpose_callback(self, msg):
        pass

    def patrol_result_callback(self, msg):
        self.patrol_result = msg.data

    def image_callback(self, msg):
        if not self.patrol_result and self.detect_QR:
            self.get_QR_message(msg)

    def get_QR_message(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8") # TODO
        # 将图像转换为灰度
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        # 通过pyzbar解码图像中的二维码
        # decoded_objects = decode(gray)
        # for obj in decoded_objects:
        #     self.QR_message.data = obj.data.decode('utf-8')
        self.detect_QR = False

    def wait_patrol_finish(self):
        while not self.patrol_result:
            time.sleep(0.5)

    def wait_detect_QR_finish(self):
        while not self.detect_QR:
                time.sleep(0.5)

    def wait_exit_keyboard_mode(self):
        while self.keyboard_mode:
            time.sleep(0.5)

    def upload_QR_messge(self):
        self.patrol_command_publisher.pulish(self.action_command)

    def order_car_to_start(self):
        self.patrol_command_publisher.pulish(self.action_command)


class Small_SUST_GTR(Node):
    def __init__(self, name, garage_number):
        super().__init__(name)
        self.get_logger().info("大家好，我是%s!" % name)
        self.garage_number = garage_number
        self.vision_subscribe_ = self.create_subscription(PerceptionTargets, "hobot_dnn_detection", self.vision_callback_, 10)
        self.cmd_vel_publisher_ = self.create_publisher(Twist, "cmd_vel", 10)
        self.parking_begin_publisher_ = self.create_publisher(Bool, "park_begin", 10)
        self.parking_car_point = [230, 230]
        self.O_DistanceThreshold = 20
        self.parking_finishing_sign = False
        self.parking_begining_sign = False
        
        self.twist = Twist()
        self.parking_begin = Bool()
        self.time_threshold = 0.01
        self.parking_time = 0
        self.parking_op_1 = [2.0, 2.0, 2.0]
        self.parking_op_2 = [5.0, 5.0, 5.0]
        self.parking_op_3 = [8.0, 8.0, 8.0]
        self.linear_x = 0.2
        self.angular_z = 0.2

    def vision_callback_(self, msg):
        # male and female flowers
        if self.parking_begining_sign:
           return 

        type_central_point = []
        if 0 != len(msg.targets):
            if msg.targets[0].type == "bottle":
                type_central_point.append(msg.targets[0].rois[0].rect.x_offset + msg.targets[0].rois[0].rect.height/2)
                type_central_point.append(msg.targets[1].rois[1].rect.x_offset + msg.targets[1].rois[1].rect.height/2)
                print(type_central_point)

    def park_car_judging(self, point):
        if self.calculate_O_distance(point) < self.O_DistanceThreshold:
            self.parking_begining_sign = True
            # 停车
            self.parking_action()

        else:
            pass


    def calculate_O_distance(self, CentralPoint):
        """ Calculate O distance. """
        return math.sqrt((self.parking_car_point[0] - CentralPoint[0])**2 + (self.parking_car_point[1] - CentralPoint[1])**2)
    
    
    def parking_action(self):
        self.parking_time = time.time()
        self.parking_begin.data = True
        self.parking_begin_publisher_.publish(self.parking_begin)
        if self.garage_number == 1:
            while not self.parking_finishing_sign:
                if time.time()- self.parking_time < self.parking_op_1[self.garage_number]:
                    self.twist.linear.x = self.linear_x
                    self.twist.linear.y = 0.0
                    self.twist.linear.z = 0.0
                    self.twist.angular.x = 0.0
                    self.twist.angular.y = 0.0
                    self.twist.angular.y = 0.0
                    self.cmd_vel_publisher_.publish(self.twist)
                elif time.time()- self.parking_time >= self.parking_op_1[self.garage_number] and time.time()- self.parking_time < self.parking_op_2[self.garage_number]:
                    self.twist.linear.x = self.linear_x
                    self.twist.linear.y = 0.0
                    self.twist.linear.z = 0.0
                    self.twist.angular.x = 0.0
                    self.twist.angular.y = 0.0
                    self.twist.angular.y = 0.1
                    self.cmd_vel_publisher_.publish(self.twist)
                elif time.time()- self.parking_time >= self.parking_op_3[self.garage_number] and time.time()- self.parking_time < self.parking_op_3[self.garage_number]:
                    self.twist.linear.x = 0.0 
                    self.twist.linear.y = 0.0
                    self.twist.linear.z = 0.0
                    self.twist.angular.x = 0.0
                    self.twist.angular.y = 0.0
                    self.twist.angular.y = self.angular_z
                    self.cmd_vel_publisher_.publish(self.twist)
                elif time.time()- self.parking_time >= self.parking_op_3[self.garage_number]:
                    self.twist.linear.x = 0.0 
                    self.twist.linear.y = 0.0
                    self.twist.linear.z = 0.0
                    self.twist.angular.x = 0.0
                    self.twist.angular.y = 0.0
                    self.twist.angular.y = 0.0
                    self.cmd_vel_publisher_.publish(self.twist)
        if self.garage_number == 2:
            while not self.parking_finishing_sign:
                if time.time()- self.parking_time < self.parking_op_1[self.garage_number]:
                    self.twist.linear.x = self.linear_x
                    self.twist.linear.y = 0.0
                    self.twist.linear.z = 0.0
                    self.twist.angular.x = 0.0
                    self.twist.angular.y = 0.0
                    self.twist.angular.y = 0.0
                    self.cmd_vel_publisher_.publish(self.twist)
                elif time.time()- self.parking_time >= self.parking_op_1[self.garage_number] and time.time()- self.parking_time < self.parking_op_2[self.garage_number]:
                    self.twist.linear.x = self.linear_x
                    self.twist.linear.y = 0.0
                    self.twist.linear.z = 0.0
                    self.twist.angular.x = 0.0
                    self.twist.angular.y = 0.0
                    self.twist.angular.y = 0.1
                    self.cmd_vel_publisher_.publish(self.twist)
                elif time.time()- self.parking_time >= self.parking_op_3[self.garage_number] and time.time()- self.parking_time < self.parking_op_3[self.garage_number]:
                    self.twist.linear.x = 0.0 
                    self.twist.linear.y = 0.0
                    self.twist.linear.z = 0.0
                    self.twist.angular.x = 0.0
                    self.twist.angular.y = 0.0
                    self.twist.angular.y = self.angular_z
                    self.cmd_vel_publisher_.publish(self.twist)
                elif time.time()- self.parking_time >= self.parking_op_3[self.garage_number]:
                    self.twist.linear.x = 0.0 
                    self.twist.linear.y = 0.0
                    self.twist.linear.z = 0.0
                    self.twist.angular.x = 0.0
                    self.twist.angular.y = 0.0
                    self.twist.angular.y = 0.0
                    self.cmd_vel_publisher_.publish(self.twist)
        if self.garage_number == 3:
            while not self.parking_finishing_sign:
                if time.time()- self.parking_time < self.parking_op_1[self.garage_number]:
                    self.twist.linear.x = self.linear_x
                    self.twist.linear.y = 0.0
                    self.twist.linear.z = 0.0
                    self.twist.angular.x = 0.0
                    self.twist.angular.y = 0.0
                    self.twist.angular.y = 0.0
                    self.cmd_vel_publisher_.publish(self.twist)
                elif time.time()- self.parking_time >= self.parking_op_1[self.garage_number] and time.time()- self.parking_time < self.parking_op_2[self.garage_number]:
                    self.twist.linear.x = self.linear_x
                    self.twist.linear.y = 0.0
                    self.twist.linear.z = 0.0
                    self.twist.angular.x = 0.0
                    self.twist.angular.y = 0.0
                    self.twist.angular.y = 0.1
                    self.cmd_vel_publisher_.publish(self.twist)
                elif time.time()- self.parking_time >= self.parking_op_3[self.garage_number] and time.time()- self.parking_time < self.parking_op_3[self.garage_number]:
                    self.twist.linear.x = 0.0 
                    self.twist.linear.y = 0.0
                    self.twist.linear.z = 0.0
                    self.twist.angular.x = 0.0
                    self.twist.angular.y = 0.0
                    self.twist.angular.y = self.angular_z
                    self.cmd_vel_publisher_.publish(self.twist)
                elif time.time()- self.parking_time >= self.parking_op_3[self.garage_number]:
                    self.twist.linear.x = 0.0 
                    self.twist.linear.y = 0.0
                    self.twist.linear.z = 0.0
                    self.twist.angular.x = 0.0
                    self.twist.angular.y = 0.0
                    self.twist.angular.y = 0.0
                    self.cmd_vel_publisher_.publish(self.twist)


def main():
    rclpy.init()
    try:
        node = Small_SUST_GTR("Small_SUST_GTR", 2)


        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node:
            node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()


