#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from functools import partial
from turtlesim.srv import Spawn
from geometry_msgs.msg import Pose2D
from example_interfaces.msg import String
import random
import math

class Spawn_turtle(Node):
    def __init__(self):
        super().__init__("spawn_turtle")    
        self.declare_parameter("spawn_sec",30)
        self.location = self.create_publisher(Pose2D, "/location", 10)
        self.name_ = self.create_publisher(String, "/turtle_name", 10)
        self.create_timer(self.get_parameter("spawn_sec").value,self.call_add2int)
        self.i=3

    def call_add2int(self):
        client = self.create_client(Spawn,"/spawn")
        while client.wait_for_service(1.0)==False:
            self.get_logger().warn("Waiting for server Spawn...")

        request=Spawn.Request()
        a=float(random.randint(1,10))
        b=float(random.randint(1,10))
        theta=round(random.uniform(0,2*math.pi),1)
        request.x=a
        request.y=b
        request.theta=theta
        request.name="turtle"+str(self.i)
        self.i+=1
        msg1=String()
        msg1.data=request.name
        self.name_.publish(msg1)

        msg=Pose2D()
        msg.x=a
        msg.y=b
        msg.theta=theta
        self.location.publish(msg)

        future = client.call_async(request)
        future.add_done_callback(partial(self.callback))

    def callback(self,future):
        try:
            response=future.result()
            self.get_logger().info(str(response.name))
        except Exception as e:
            self.get_logger().error("Service call failed %r" % (e,))


def main(args=None):
    rclpy.init(args=args)
    node = Spawn_turtle()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
        main()  
