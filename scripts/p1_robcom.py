#! /usr/bin/env python3
# -*- coding:utf-8 -*-
from __future__ import print_function, division
import rospy
import numpy as np
import numpy
import tf
import math
import cv2
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge, CvBridgeError
from numpy import linalg
from tf import transformations
from tf import TransformerROS
import tf2_ros
from geometry_msgs.msg import Twist, Vector3, Pose, Vector3Stamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Header
import visao_module


"""
Arquivo para trabalharmos usando classe e orientação a objetos:
"""

# AMARELO_FAIXA = np.array([255,255,0])
# VERDE_CREEP = np.array([1,255,2])
# AZUL_CREEP = np.array([98, 227, 255])
# LARANJA_CREEP = np.array([255, 16, 0])

class Robot:
    def __init__(self):
        """
        Cria variáveis necessárias para o funcionamento do robô
        ou para armazenamento de dados 
        """
        self.bridge = CvBridge()
        rospy.init_node("cor")
        
        self.topico_imagem = "/camera/image/compressed"
        self.recebedor = rospy.Subscriber(self.topico_imagem, CompressedImage, self.trata_frame, queue_size=4, buff_size=1)
        self.velocidade_saida = rospy.Publisher("/cmd_vel", Twist, queue_size=1)

        #! Variáveis para guardar filtros de interesse
        self.visao_creeper = None
        self.visao_trilha = None 
        self.visao_aruco = None

        self.STATUS = {
            'trilhaON': True,
            'arucoON': False,
            'searchCreepON': True,
            'searchBaseON': False,
        }

        #% Não sei oq fazem:
        # tfl = tf2_ros.TransformListener(tf_buffer) #conversao do sistema de coordenadas 
        # tolerancia = 25

    def run(self):
        """
        Função que chama os métodos principais e os mantém em loop 
        """
        try: 
            while not rospy.is_shutdown():
                #<> Usar STATUS p/ movimentação

                if self.STATUS['trilhaON']:
                #$ if TrilhaON:
                    # --> Faz o robô percorrer a trilha no sentido que combinamos
                    vel = Twist(Vector3(0.1,0,0), Vector3(0,0,0))
                    self.velocidade_saida.publish(vel)

                if self.STATUS['searchCreepON']:
                #$ if SearchCreepON:
                    # --> Faz o robô procurar o creeper da cor recebida    
                    # --> Se achar: (1) - Marca posição; (2) - Vai até ele e pega
                    print("Cadê o creeper?")
                    pass

                if self.STATUS['searchBaseON']:
                #$ if SearchBaseON:
                    # --> Faz o robô procurar pelas estações
                    # --> Se achar: Marca posição
                    pass

                rospy.sleep(0.1)


        except rospy.ROSInterruptException:
            print("Ocorreu uma exceção com o rospy")

    def trata_frame(self, frame):
        """ 
        Função assíncrona de Callback, é chamada toda vez pelo Subscriber 
        """
        # Trata delay:

        # now = rospy.get_rostime()
        # imgtime = frame.header.stamp
        # lag = now-imgtime # calcula o lag
        # delay = lag.nsecs
        # # print("delay ", "{:.3f}".format(delay/1.0E9))
        # if delay > atraso and check_delay==True:
        #     # Esta logica do delay so' precisa ser usada com robo real e rede wifi 
        #     # serve para descartar imagens antigas
        #     print("Descartando por causa do delay do frame:", delay)
        #     return 

        
        try:
            #$ Exibe Imagem Original
            cv_image_original = self.bridge.compressed_imgmsg_to_cv2(frame, "bgr8")
            cv2.imshow("Camera", cv_image_original)

            if self.STATUS['trilhaON']:
                #$ Segmenta Amarelo (Trilha) 
                #>> apenas para visualização (não é necessário exibir a imagem toda vez)
                segmentado_trilha = self.segmenta_cor(cv_image_original, "amarelo")
                cv2.imshow("seg_trilha", segmentado_trilha)
                cv2.waitKey(1)

            if self.STATUS['searchCreepON']:
                #$ Segmenta Creeper
                #>> apenas para visualização (não é necessário exibir a imagem toda vez)
                """
                Para que a imagem do Creeper não atrapalhe na detecção da linha, 
                acredito que tenhamos que analisar os filtros em imagens diferentes.  
                """
                segmentado_creeper = self.segmenta_cor(cv_image_original, "verde")  #! Depois, vamos automatizar para escolher a cor da missão
                cv2.imshow("seg_creeper", segmentado_creeper)
                cv2.waitKey(1)
            
            # cv2.waitKey(1)

        except CvBridgeError as e:
            print('ex', e)      

    def run_mobileNet(self):
        pass
    
    def draw_crossHair(self):
        # Desenha + no meio da tela
        pass

    def segmenta_cor(self, frame, COR):
        output = frame.copy()
        if COR == 'amarelo':
            bgr_min = np.array([0, 50, 50], dtype=np.uint8)
            bgr_max = np.array([0,255,255], dtype=np.uint8)
            segmentado_cor = cv2.inRange(output, bgr_min, bgr_max)

        elif COR == 'verde':
            bgr_min = np.array([0, 50, 0], dtype=np.uint8)
            bgr_max = np.array([5,255,5], dtype=np.uint8)
            segmentado_cor = cv2.inRange(output, bgr_min, bgr_max)

        elif COR == 'azul':
            # Azul(Malibu)-HSV 360: [191,100,69] --> 180: [95, 50, 35]
            #! Tratar como HSV:
            output = cv2.cvtColor(output, cv2.COLOR_BGR2HSV)
            hsv_min = np.array([93, 40, 28], dtype=np.uint8)
            hsv_max = np.array([98, 255,255], dtype=np.uint8)
            segmentado_cor = cv2.inRange(output, hsv_min, hsv_max)

        elif COR == 'laranja':
            # Laranja-HSV 360: [4, 100, 50] --> 180: [2,50,25]
            #! Tratar como HSV:
            output = cv2.cvtColor(output, cv2.COLOR_BGR2HSV)
            hsv_min = np.array([0, 50, 50], dtype=np.uint8)
            hsv_max = np.array([4, 255,255], dtype=np.uint8)
            segmentado_cor = cv2.inRange(output, hsv_min, hsv_max)

        # Aplica Morphology
        segmentado_cor = cv2.morphologyEx(segmentado_cor,cv2.MORPH_CLOSE,np.ones((7, 7)))
        
        return segmentado_cor

    def encontra_trilha(self):
        # Trabalhar com a imagem segmentado do amarelo
        # Para descobrir direção e fazer seguidor de linha
        pass

    def percorre_trilha(self):
        #! Etapa 1: fazer o robô percorrer a trilha no sentido que adotamos (virar à direita)
        
        #! Etapa 2: configurar para que ele acelere qnd estiver em linha reta
        pass
