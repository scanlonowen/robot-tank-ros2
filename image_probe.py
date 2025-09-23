import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2, numpy as np
from geometry_msgs.msg import Twist

class ImageProbe(Node):
    def __init__(self):
        super().__init__('image_probe')
        self.sub = self.create_subscription(Image, '/image_raw', self.image_cb, 10)

        self.mask_pub    = self.create_publisher(Image, 'mask', 10)
        self.overlay_pub = self.create_publisher(Image, 'overlay', 10)   # NEW
        self.cmd_pub     = self.create_publisher(Twist, '/cmd_vel', 10)

        self.low1  = np.array([  0, 120, 70], dtype=np.uint8)
        self.high1 = np.array([ 10, 255,255], dtype=np.uint8)
        self.low2  = np.array([170, 120, 70], dtype=np.uint8)
        self.high2 = np.array([180, 255,255], dtype=np.uint8)

        self.bridge = CvBridge()
        self.kernel = np.ones((5,5), np.uint8)

        self.declare_parameter('target_frac', 0.020)
        self.declare_parameter('tol_frac',    0.004)
        self.declare_parameter('turn_gain',   1.2)

    def image_cb(self, msg: Image):
        target_frac = float(self.get_parameter('target_frac').value)
        tol_frac    = float(self.get_parameter('tol_frac').value)
        turn_gain   = float(self.get_parameter('turn_gain').value)

        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        h, w = frame.shape[:2]
        center_x = w // 2

        hsv   = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask1 = cv2.inRange(hsv, self.low1, self.high1)
        mask2 = cv2.inRange(hsv, self.low2, self.high2)
        mask  = cv2.bitwise_or(mask1, mask2)
        mask  = cv2.morphologyEx(mask, cv2.MORPH_OPEN,  self.kernel)
        mask  = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, self.kernel)

        self.mask_pub.publish(self.bridge.cv2_to_imgmsg(mask, encoding='mono8'))

        vx, wz = 0.0, 0.0
        cnts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if cnts:
            c = max(cnts, key=cv2.contourArea)
            area = float(cv2.contourArea(c))
            M = cv2.moments(c)
            if M["m00"] > 0:
                cx = int(M["m10"]/M["m00"])
                cy = int(M["m01"]/M["m00"])   # NEW

                # overlays (NEW)
                cv2.circle(frame, (cx, cy), 8, (0,255,0), -1)                # centroid
                cv2.drawContours(frame, [c], -1, (255,0,0), 2)               # contour
                cv2.line(frame, (center_x,0), (center_x,h), (0,255,255), 2)  # center line

                # control
                err_x = (cx - center_x) / float(center_x)
                if abs(err_x) < 0.05:
                    err_x = 0.0
                wz = float(np.clip(-turn_gain * err_x, -1.0, 1.0))

                img_area  = float(h * w)
                area_frac = area / img_area
                err_frac  = target_frac - area_frac
                if abs(err_frac) < tol_frac:
                    vx = 0.0
                else:
                    vx = float(np.clip(err_frac / target_frac, -1.0, 1.0))

        tw = Twist()
        tw.linear.x  = vx
        tw.angular.z = wz
        self.cmd_pub.publish(tw)

        # NEW: publish overlay image
        self.overlay_pub.publish(self.bridge.cv2_to_imgmsg(frame, encoding='bgr8'))

        self.get_logger().info(f"cmd(vx={vx:.2f}, wz={wz:.2f})")

def main():
    rclpy.init()
    node = ImageProbe()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

