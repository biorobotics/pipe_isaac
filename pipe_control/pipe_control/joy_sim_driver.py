#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist

MAX_LIN_VEL = 10.0 
MAX_ANG_VEL = 3.0

class JoystickControllerNode(Node):
    def __init__(self):
        super().__init__('diff_drive_teleop_sim')
        
        self._joy_sub = self.create_subscription(Joy, "/joy", self.joy_callback, 10)
        self.cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)

        timer_period = 1 / 100
        self._timer = self.create_timer(timer_period, self._cmd_callback,)

        self.forward_input = 0.0        # forward/backward control
        self.rotation_input = 0.0       # rotation control 
        self.speed_multiplier = 0.0     # deadman switch
        

    def joy_callback(self, msg):
        if msg.buttons[6] == 0:
            self.speed_multiplier = 0.0
        else:
            self.speed_multiplier = 1.0

        self.forward_input = msg.axes[1]
        
        self.rotation_input = msg.axes[2]

        if self.forward_input and self.rotation_input:
            cmd_vel = Twist()
            
            cmd_vel.linear.x = -1.0 * self.forward_input * self.speed_multiplier * MAX_LIN_VEL
            
            cmd_vel.angular.z = -1.0 * self.rotation_input * self.speed_multiplier * MAX_ANG_VEL
            
            self.cmd_vel_pub.publish(cmd_vel)


def main(args=None):
    rclpy.init(args=args)
    joy_controller = JoystickControllerNode()
    try:
        rclpy.spin(joy_controller)
    except KeyboardInterrupt:
        pass
    finally:
        joy_controller.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()