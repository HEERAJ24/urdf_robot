#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose2D
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import math
from turtlesim.srv import Kill
from functools import partial
from example_interfaces.msg import String

class Turtle_catch(Node):
    def __init__(self):
        super().__init__("turtle_catch")   
        self.location = self.create_subscription(Pose2D,"/location",self.call_back,10)
        self.cmd_vel_pub = self.create_publisher(Twist,"/turtle1/cmd_vel",10)
        self.prime_location = self.create_subscription(Pose,"/turtle1/pose",self.call_back2,10)
        self.location_=None
        self.turtle_name_ = self.create_subscription(String,"/turtle_name",self.call_back3,10)
        self.row=["","turtle3"]

    def call_back(self,msg):
        self.location_=msg

    def call_back2(self,msg1):
        self.flag=0
        if msg1==None or self.location_==None:
            return
        x_prime=msg1.x
        y_prime=msg1.y
        theta_prime=msg1.theta
        msg2 = Twist()
        distance = math.sqrt((x_prime - self.location_.x)**2 + (y_prime - self.location_.y)**2)
        angle_to_target = math.atan2(self.location_.y - y_prime, self.location_.x - x_prime)

        angle_difference = angle_to_target - theta_prime

        if distance > 0.5: 
            if angle_difference > math.pi:
                angle_difference -= 2 * math.pi
            
            elif angle_difference < -math.pi:
                angle_difference += 2 * math.pi

            msg2.linear.x = 2*distance 
            msg2.angular.z = 6*angle_difference
            self.cmd_vel_pub.publish(msg2)
            
        else:
            msg2.linear.x = 0.0
            msg2.angular.z = 0.0
            self.cmd_vel_pub.publish(msg2)
            self.flag=1

    def call_back3(self,msg):
        if self.flag==1:
            self.row[0]=self.row[1]
            self.row[1]=msg.data
            client = self.create_client(Kill,"/kill")
            while client.wait_for_service(1.0)==False:
                self.get_logger().warn("Waiting for server Spawn...")
            request=Kill.Request()
            request.name=self.row[0]

            future = client.call_async(request)
            future.add_done_callback(partial(self.callback))
            self.flag=0
            self.get_logger().info(str(request.name)+" has been killed")
        else:
            pass
    
    def callback(self,future):
        pass
        

def main(args=None):
    rclpy.init(args=args)
    node = Turtle_catch()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
        main()  
