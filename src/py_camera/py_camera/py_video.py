import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from std_msgs.msg import String
import cv2
import numpy as np
import time


class imageReciever(Node):
    def __init__(self):
        super().__init__('image_reciever')
        self.publisher = self.create_publisher(String, 'camera', 2)
        self.subscriber = self.create_subscription(Image, 'videoFrames', self.listener_callback, 30)
        self.subscriber
        self.br = CvBridge()
        
    def listener_callback(self,data):
        self.get_logger().info('Recieving video frame')
        frame = self.br.imgmsg_to_cv2(data)
        
        # Get the height and width of the frame
        height, width = frame.shape[:2]
        
        # Draw a circle at the center of the frame
        center_x = int(width / 2) + 55
        center_y = int(height / 2)
        radius = 5
        cv2.circle(frame, (center_x, center_y), radius, (0, 0, 0), 3)
        
        # convert the frame to HSV color space
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        
        # define range of red color in HSV
        lower_red = np.array([130, 80, 30], dtype=np.uint8)
        upper_red = np.array([179, 255, 255], dtype=np.uint8)
        mask_red = cv2.inRange(hsv, lower_red, upper_red)
        
        # combine the masks
        mask = mask_red
        
        # apply the mask to the original frame
        res = cv2.bitwise_and(frame, frame, mask=mask)
        
        # convert the masked frame to grayscale
        gray = cv2.cvtColor(res, cv2.COLOR_BGR2GRAY)
        
        # detect edges using Canny edge detection
        edges = cv2.Canny(gray, 50, 150)
        
        # find contours of the object
        contours, hierarchy = cv2.findContours(edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        for cnt in contours:
            # fit a rectangle around the object
            x, y, w, h = cv2.boundingRect(cnt)
            area = w * h
            aspect_ratio = float(w) / h
            
            # check if the object is square-shaped and has an area greater than 12
            if aspect_ratio >= 0.5 and aspect_ratio <= 1.1 and area > 2500 and area < 8000:
                mask_roi = mask[y:y+h, x:x+w]
                mean_color = cv2.mean(hsv[y:y+h, x:x+w], mask=mask_roi)[:3]
                if mean_color[0] >= lower_red[0] and mean_color[0] <= upper_red[0]:
                    color = 'red'
                    cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 0, 255), 2) # draw a red rectangle around the object
                    print('red')
                    # Find the centroid of the rectangle
                    cx = int(x + w/2)
                    cy = int(y + h/2)

                    # Draw a small circle at the centroid
                    cv2.circle(frame, (cx, cy), 3, (0, 0, 255), -1)
                    
                    #Directions to get to the center of the frame
                    if(cx > center_x + 20):
                        message = "right"
                        msg = String()
                        msg.data = '%s' % message
                        self.publisher.publish(msg)
                    elif(cx < center_x - 20):
                        message = "left"
                        msg = String()
                        msg.data = '%s' % message
                        self.publisher.publish(msg)
                    elif(cy > center_y + 20):
                        message = "backwards"
                        msg = String()
                        msg.data = '%s' % message
                        self.publisher.publish(msg)
                    elif(cy < center_y - 20):
                        message = "forward"
                        msg = String()
                        msg.data = '%s' % message
                        self.publisher.publish(msg)
                    else:
                        message = "center"
                        msg = String()
                        msg.data = '%s' % message
                        self.publisher.publish(msg)
                        time.sleep(15)

        cv2.imshow("camera", frame)
        cv2.waitKey(1)
        
def main(args=None):
    rclpy.init(args=args)
    image_reciever = imageReciever()
    
    rclpy.spin(image_reciever)
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()
