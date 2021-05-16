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
import aruco1

"""
Arquivo para trabalharmos usando classe e orientação a objetos:
"""
goal = ("blue", 12, "dog")

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
        self.recebedor = rospy.Subscriber(self.topico_imagem, CompressedImage, self.trata_frame, queue_size=1, buff_size=2**24)
        self.velocidade_saida = rospy.Publisher("/cmd_vel", Twist, queue_size=1)

        self.output_img = None
        self.CENTRO_ROBO = None

        #! Variáveis para guardar filtros de interesse
        self.visao_creeper = None
        self.visao_trilha = None 
        self.visao_aruco = None

        self.STATUS = {
            'trilhaON': True,
            'arucoON': False,
            'searchCreepON': True,
            'searchCreepDetected': False,
            'searchCreepConfirmed': False,
            'searchBaseON': False,
        }

        self.LOCALIZACOES = {
            'creeper': None,
            'base': None,
        }

        self.ALVO = {
            'centro': None,
            'distancia': None,
        }

        self.ID = {
            'ids': None,
            'comparado': 0,
            'match': 0,
        }

        #% Não sei oq fazem:
        # tfl = tf2_ros.TransformListener(tf_buffer) #conversao do sistema de coordenadas 
        # tolerancia = 25
        

        #! LOOPING PRINCIPAL
        r = rospy.Rate(100)
        while not rospy.is_shutdown():
            self.run()
            r.sleep() 

    def run(self):
        """
        Função que chama os métodos principais e os mantém em loop 
        """
        try: 
            #<> Usar STATUS p/ MOVIMENTAÇÃO

            if self.STATUS['trilhaON']:
            #$ if TrilhaON:
                # --> Faz o robô percorrer a trilha no sentido que combinamos
                vel = Twist(Vector3(0.15,0,0), Vector3(0,0,0))
                self.velocidade_saida.publish(vel)

            else:
                vel = Twist(Vector3(0.0,0,0), Vector3(0,0,0))
                self.velocidade_saida.publish(vel)

            if self.STATUS['searchCreepDetected']:
            #$ if SearchCreepDetected:
                # --> Faz o robô confirmar se o creeper avistado é da id desejada
                # --> Se achar: (1) - Marca posição; (2) - Vai até ele e pega
                self.centraliza_robo(self.ALVO['centro'])

            if self.STATUS['searchBaseON']:
            #$ if SearchBaseON:
                # --> Faz o robô procurar pelas estações
                # --> Se achar: Marca posição
                pass


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
            #$ Exibe Imagem Originaloutput_img
            cv_image_original = self.bridge.compressed_imgmsg_to_cv2(frame, "bgr8")
            cv2.imshow("Camera", cv_image_original)
            cv2.waitKey(1)
            self.CENTRO_ROBO = (cv_image_original.shape[1]//2, cv_image_original.shape[0]//2)
            self.output_img = cv_image_original.copy()

            # if self.STATUS['trilhaON']:
            #     #$ Segmenta Amarelo (Trilha) 
            #     #>> apenas para visualização (não é necessário exibir a imagem toda vez)
            #     segmentado_trilha = self.segmenta_cor(cv_image_original, "amarelo")
            #     cv2.imshow("seg_trilha", segmentado_trilha)
            #     cv2.waitKey(1)

            if self.STATUS['searchCreepON']:
                #$ Segmenta Creeper
                #>> apenas para visualização (não é necessário exibir a imagem toda vez)
                """
                Para que a imagem do Creeper não atrapalhe na detecção da linha, 
                acredito que tenhamos que analisar os filtros em imagens diferentes.  
                """
                segmentado_creeper = self.segmenta_cor(cv_image_original, "azul")  #! Depois, vamos automatizar para escolher a cor da missão
                self.calcula_area(segmentado_creeper)  # > 800
                cv2.imshow("seg_creeper", segmentado_creeper)

            if self.STATUS["arucoON"]:
                self.localiza_id(frame)


                #! CALCULA ÁREA --> se maior q X -> searchCreepDetected =True --> centraliza no creeper e aproxima.
            
            # cv2.imshow("Camera", self.output_img)
            # cv2.waitKey(0)
            # cv2.destroyAllWindows()

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

    def calcula_area(self, segmentado):
        contornos, arvore = cv2.findContours(segmentado.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE) 

        maior_contorno = None
        maior_area = 0

        for cnt in contornos:
            area = cv2.contourArea(cnt)
            if area > maior_area:
                maior_contorno = cnt
                maior_area = area
        
        if maior_area > 1000:
            self.STATUS['searchCreepDetected'] = True  #! Faz o robô centralizar E aproximar do creeper
            self.STATUS['trilhaON'] = False #! Faz o robô parar de seguir linha
            print(f"Creeper avistado -- {maior_area}")
            print(f" trilha: {self.STATUS['trilhaON']}")

        if not maior_contorno is None:
            media = maior_contorno.mean(axis=0)
            media = media.astype(np.int32)
            # Desenha um circulo no centro do creeper
            # cv2.circle(self.output_img, (media[0], media[1]), 5, [100, 255, 100])

            # Guarda a posição do centro em ALVOS:
            self.ALVO['centro'] = media[0] #(x,y)


        #! DESENHA O CENTRO DA ÁREA NA IMG ORIGINAL

    def centraliza_robo(self, alvo):
        # Centraliza o robô, apontando para o creeper
        print("Centralizar robô com o Alvo")


        while (abs(alvo[0] - self.CENTRO_ROBO[0]) >= 20):
            if (alvo[0] > self.CENTRO_ROBO[0]):
                # Gira p/ direita
                vel = Twist(Vector3(0.0,0,0), Vector3(0,0,-0.1))
                self.velocidade_saida.publish(vel)
            else:
                # Gira p/ esquerda 
                vel = Twist(Vector3(0.0,0,0), Vector3(0,0,0.1))
                self.velocidade_saida.publish(vel)

        # PARAR
        vel = Twist(Vector3(0.0,0,0), Vector3(0,0,0))
        self.velocidade_saida.publish(vel)
        print("Centralizado")
        self.STATUS['arucoON'] = True

    def localiza_id(self, frame):
        try:
            #cv_image_original = self.bridge.compressed_imgmsg_to_cv2(frame, "bgr8")
            #aruco_image = cv_image_original.copy()
            ids = aruco1.roda_todo_frame(frame)
            
            self.ID['comparado'] = ids[0][0]
            
            if self.ID['match'] > 200:
                self.STATUS['searchCreepConfirmed'] = True
                self.STATUS['searchCreepDetected'] = False
                print('achou')

            # Se for falso, volta para a pista:
            else:
                print('procurando')
                #self.ID['match'] = 0

            if self.ID['comparado'] == goal[1]:
                print('opa')
                self.ID['match'] += 1
        except CvBridgeError as e:
            print('ex', e)  

    def encontra_trilha(self):
        # Trabalhar com a imagem segmentado do amarelo
        # Para descobrir direção e fazer seguidor de linha
        pass

    def percorre_trilha(self):
        #! Etapa 1: fazer o robô percorrer a trilha no sentido que adotamos (virar à direita)
        
        #! Etapa 2: configurar para que ele acelere qnd estiver em linha reta
        pass
