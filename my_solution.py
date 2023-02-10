import rclpy
import cv2
import numpy as np

from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
from rclpy.qos import QoSProfile
from rclpy.qos import QoSReliabilityPolicy, QoSHistoryPolicy

# SPEED = 20.0
SPEED = 10.0

qos_profile = QoSProfile(
    reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
    history=QoSHistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
    depth=1)

redBajo1 = np.array([0, 100, 20], np.uint8)
redAlto1 = np.array([30, 255, 255], np.uint8)

im_width = 0
im_height = 0

last_error = 0
last_mode = 0

def filter_red_line(frame):
    frameHSV = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)        
    maskRed = cv2.inRange(frameHSV, redBajo1, redAlto1)
    cv2.imshow('frame', maskRed)
    return maskRed


def control_yaw(frame,current_frame):
    global last_error,last_mode,SPEED
    # print(np.shape(frame))
    row = int (0.53 * im_height)
    # print(im_height,im_width)
    see_line = frame[row,:]
    print(row, len(see_line))
    see_line = list(see_line)
    begin = see_line.index(255)
    see_line.reverse()
    end=len(see_line) - see_line.index(255)
    # print(begin,end)
    medium = int((end+begin)/2)
    medium_ref = im_width/2
    error = medium_ref - medium
    image = cv2.circle(current_frame, (medium,row), 5, (250,0,0),-1)
    print(f'{error=}')
    msg = Twist()
    x_speed = SPEED
    msg.linear.x = x_speed
    if abs(error) < 20:
        mode = 0
        msg.angular.z = float (error /70)
    elif abs(error) < 40:
        mode = 1
        msg.angular.z = float (error /60)
    elif abs(error) < 50:
        mode = 2
        msg.angular.z = float (error /50)
    elif abs(error) < 70:
        mode = 3
        msg.angular.z = float (error /30)
    else:
        mode = 4
        msg.angular.z = float (error /15)

    if mode <2 and last_mode >=3:
    #     print('converging')
         msg.angular.z *= 0.8

    # if abs(last_error - error) > 30 and error < 30 :
    #     print('converging')
    #     msg.angular.z *= 0.8

    last_error = error
    last_mode = mode

    return msg
    



class Pilot(Node):

    def __init__(self):
        super().__init__('miguel_pilot')
        self.subscription = self.create_subscription(
            Image,
            'cam_f1/image_raw',
            self.listener_callback,
            qos_profile)
        self.subscription  # prevent unused variable warning
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.br = CvBridge()

    def listener_callback(self, msg: Image):
        current_frame = self.br.imgmsg_to_cv2(msg)
        global im_height, im_width
        im_width = msg.width
        im_height = msg.height
        current_frame = cv2.cvtColor(current_frame,cv2.COLOR_BGR2RGB)
        line_filtered = filter_red_line(current_frame)
        try:
            out = control_yaw(line_filtered,current_frame)
            self.publisher_.publish(out)
            cv2.imshow('image',current_frame)
        except:
            cv2.imshow('image',current_frame)
            cv2.waitKey(20)
            raise Exception('Error')
            pass
        cv2.waitKey(20)
        # self.get_logger().info('I heard: "%s"' % msg.data)

def main(args=None):
    rclpy.init(args=args)

    node = Pilot()

    rclpy.spin(node)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
