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
        super().__init__('image_projector')

        # Parâmetro da pasta com imagens
        self.declare_parameter('image_path', '/home/jetson/Videos/random_pattern')
        image_path = self.get_parameter('image_path').get_parameter_value().string_value

        # Carrega todas as imagens da pasta
        self.image_files = sorted(glob(os.path.join(image_path, '*.png')))
        if not self.image_files:
            self.get_logger().error(f'Nenhuma imagem encontrada em {image_path}')
            return

        self.index = 0

        # Serviço para mudar índice da imagem
        self.srv = self.create_service(Trigger, 'next_image', self.change_image_callback)

        # Nome da janela OpenCV
        self.window_name = 'Image Projector'
        cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
        cv2.setWindowProperty(self.window_name, cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)

        self.get_logger().info(f'Iniciando projeção de {len(self.image_files)} imagens.')
        self.timer = self.create_timer(0.1, self.timer_callback)  # Atualiza a tela a cada 100ms

    def timer_callback(self):
        img = cv2.imread(self.image_files[self.index],0)
        img_c = np.zeros((1080, 1920, 3), dtype=np.uint8)
        img_c[:,:,0] = img[:1080,:]
        if img is not None:
            cv2.imshow(self.window_name, img_c)
            cv2.waitKey(10)

    def change_image_callback(self, request, response):
        # Incrementa índice e faz loop
        self.index = (self.index + 1) % len(self.image_files)
        self.get_logger().info(f'Mostrando imagem {self.index}: {self.image_files[self.index]}')
        response.success = True
        response.message = f'Imagem atual: {self.image_files[self.index]}'
        return response


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
