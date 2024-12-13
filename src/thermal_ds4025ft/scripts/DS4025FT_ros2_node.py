#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Int32MultiArray, Float32
from cv_bridge import CvBridge

import cv2

# from package name.module import class
from thermal_ds4025ft.ds4025ft import Thermal_DS4025FT



class DS4025FT_ros2_node(Node):

    def __init__(self):
        super().__init__('thermal_camera_node')
        
        self.account = "admin"
        self.password = "admin"
        self.ip_address = "192.168.1.108"


        self.thermal_camera = Thermal_DS4025FT(ip_address=self.ip_address, account=self.account, password=self.password)

        self.vcap = self.thermal_camera.getThermalStream()

        self.bridge = CvBridge()

        self.max_temperature_pos_msg = Int32MultiArray()

        self.max_temperature_msg = Float32()

        
        self.image_pub = self.create_publisher(Image, 'thermal_image', 10)

        self.max_temperature_pixel_pub = self.create_publisher(Int32MultiArray, 'hot_spot_temperature_pos', 10)
        self.max_temperature_pub = self.create_publisher(Float32, 'hot_spot_temperature', 10)


        self.publish_temperature_pos = self.create_timer(1, self.pos_temp_callback)
        self.publish_image = self.create_timer(0.001, self.image_callback)


    def pos_temp_callback(self):

            max_temperature, max_temperature_pixel = self.thermal_camera.getHostTemperatureAndPosition()
            


            self.get_logger().info('Max Temperature: {} Celsius'.format(max_temperature))
            self.get_logger().info('Max Temperature Pixel: {}'.format(max_temperature_pixel))

            
            self.max_temperature_pos_msg.data = [int(i) for i in max_temperature_pixel]
            self.max_temperature_pixel_pub.publish(self.max_temperature_pos_msg)

            
            self.max_temperature_msg.data = max_temperature
            self.max_temperature_pub.publish(self.max_temperature_msg)
        # else:
        #     self.max_temperature_pos_msg.data = [-1, -1]
        #     self.max_temperature_pixel_pub.publish(self.max_temperature_pos_msg)

            
        #     self.max_temperature_msg.data = -100.0
        #     self.max_temperature_pub.publish(self.max_temperature_msg)




    def image_callback(self):

        
        ret, frame = self.vcap.read()    
        if ret:
            # frame = cv2.resize(frame, (640, 480))
            img_msg = self.bridge.cv2_to_imgmsg(frame, "bgr8")
            # self.get_logger().info('Publishing: Thermal Image')
            self.image_pub.publish(img_msg)
        else:
            print('[Error] Failed to read frame from camera')






def main(args=None):


    rclpy.init(args=args)
    node = DS4025FT_ros2_node()
    rclpy.spin(node)



    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()






if __name__ == '__main__':
    main()





