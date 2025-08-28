#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
import cv2
import os
from glob import glob
import numpy as np
class ImageProjector(Node):
    def __init__(self):
        super().__init__('image_white_node')
        # Nome da janela OpenCV
        self.window_name = 'Image Projector'
        cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
        cv2.setWindowProperty(self.window_name, cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)

        self.timer = self.create_timer(10, self.timer_callback)  # Atualiza a tela a cada 100ms

    def timer_callback(self):
        img = np.ones((1080, 1920,3), dtype=np.uint8)*255
        if img is not None:
            cv2.imshow(self.window_name, img)
            cv2.waitKey(10)

def main(args=None):
    rclpy.init(args=args)
    node = ImageProjector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    cv2.destroyAllWindows()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
