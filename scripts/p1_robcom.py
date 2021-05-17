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
        self.recebedor = rospy.Subscriber(self.topico_imagem, CompressedImage, self.trata_frame, queue_size=1, buff_size=2**24)
        self.velocidade_saida = rospy.Publisher("/cmd_vel", Twist, queue_size=1)

        self.output_img = None
        self.CENTRO_ROBO = None
        self.tolerancia_giro = 20

        #! Variáveis para guardar filtros de interesse
        self.visao_creeper = None
        self.visao_trilha = None 
        self.visao_aruco = None

        self.STATUS = {
            'trilhaON': True,                   # faz o robô percorrer a trilha
            'searchTrilha': False,              # faz o robô girar à direita até achar uma trilha
            'arucoON': False,                   # ativa o leitor do aruco
            'searchCreepON': True,              # ativa a detecção do creeper em função da cor
                'searchCreepDetected': False,       # detectou massa de creeper na imagem --> centraliza no Creeper
                'searchCreepConfirmed': False,      # searchCreep detectou q É o id correto: --> pega o Creeper
                'searchCreepMistaked': False,       # searchCreep detectou q não é o id correto: --> volta para trilha
                    #! limpar dados em self.ALVO
            'searchBaseON': False,              # ativa o mobileNet para identificar a base
            'checkpoint': False,                # variável utilizada para gravar informações pontualmente (sem repetições)
        }

        self.LOCALIZACOES = {
            'creeper': None,    # guarda localizacao do Ponto em que o creeper é identificado
            'base': None,       # guarda localizacao do Ponto em que a base é identificada
        }

        self.ALVO = {
            'centro': None,         # guarda o centro do Alvo (base ou creeper)
            'distancia': None,      # guarda a distância do Alvo
            'sentidoGiro': None,    # guarda o sentido inicial do giro para centralizar no alvo
            'avanco': None,         # guarda se o robô avançou ou não até o creep
        }

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
                pass

            # if self.STATUS["searchTrilha"]:
                # --> Faz robô girar para a direita procurando a pista
                #! ADAPTAR PARA GIRAR NO SENTIDO CONTRÁRIO AO DE GIRO
                #! PARA O CASO EM QUE CENTRALIZA NO ALVO
                # self.gira_direita()

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

            if self.STATUS['arucoON']:
                self.STATUS['searchCreepDetected'] = False
                self.STATUS['searchCreepMistaked'] = True
                self.STATUS['arucoON'] = False

            
            if self.STATUS['searchCreepMistaked'] and not self.STATUS['trilhaON']:
                self.retorna_para_trilha() # faz girar no sentido contrário ao self.ALVO['sentidoGiro']
                #! IDENTIFICAR TRILHA 
                #! --> qnd identificado --> para de girar --> ativa TRILHA ON e desativa MISTAKED
                #! DESATIVAR o searchCreepON e ativá-lo após alguns segundos
            if self.STATUS['searchCreepConfirmed']:
                # self.captura_creep()
                # self.vel_frente() #faz o robô avançar

                #TODO: Fazer o robô PEGAR o Creep e VOLTAR para pista
                pass
        except rospy.ROSInterruptException:
            print("Ocorreu uma exceção com o rospy")

    def trata_frame(self, frame):
        global HEIGHT, WIDTH
        """ 
        Função assíncrona de Callback, é chamada toda vez pelo Subscriber 
        """

        try:
            #$ Exibe Imagem Originaloutput_img
            cv_image_original = self.bridge.compressed_imgmsg_to_cv2(frame, "bgr8")
            # cv2.imshow("Camera", cv_image_original)
            # cv2.waitKey(1)
            self.CENTRO_ROBO = (cv_image_original.shape[1]//2, cv_image_original.shape[0]//2)
            self.output_img = cv_image_original.copy()

            HEIGHT = cv_image_original.shape[0]
            WIDTH = cv_image_original.shape[1]

            if self.STATUS['trilhaON']:
                #$ Segmenta Amarelo (Trilha) e faz robô andar a trilha
                self.percorre_trilha(self.output_img)
                cv2.waitKey(1)

            if self.STATUS["searchTrilha"]:
                #$ Permanece girando até que encontre a pista
                self.encontra_contornos(self.segmenta_cor(self.output_img, 'amarelo'))

            if self.STATUS['searchCreepON']:
                #$ Segmenta Creeper
                #>> apenas para visualização (não é necessário exibir a imagem toda vez)
                """
                Para que a imagem do Creeper não atrapalhe na detecção da linha, 
                acredito que tenhamos que analisar os filtros em imagens diferentes.  
                """
                segmentado_creeper = self.segmenta_cor(cv_image_original, "verde")  #! Depois, vamos automatizar para escolher a cor da missão
                self.calcula_area(segmentado_creeper)  # > 800
                cv2.imshow("seg_creeper", segmentado_creeper)
                cv2.waitKey(1)

                #! CALCULA ÁREA --> se maior q X -> searchCreepDetected =True --> centraliza no creeper e aproxima.        
            



        except CvBridgeError as e:
            print('ex', e)      

    def run_mobileNet(self):
        pass
    
    def draw_crossHair(self, img, point, size, color):
        # Desenha + no meio da tela
        x,y = point
        cv2.line(img,(x - size,y),(x + size,y),color,2)
        cv2.line(img,(x,y - size),(x, y + size),color,2)

    def segmenta_cor(self, frame, COR):
        output = frame.copy()
        if COR == 'amarelo':
            bgr_min = np.array([0, 190, 190], dtype=np.uint8)
            bgr_max = np.array([50, 255, 255], dtype=np.uint8)
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
        segmentado_cor = cv2.morphologyEx(segmentado_cor,cv2.MORPH_CLOSE,np.ones((5, 5)))
        
        return segmentado_cor

    #@ chamada qnd searchCreepON = True
    def calcula_area(self, segmentado):
        contornos, arvore = cv2.findContours(segmentado.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE) 

        maior_contorno = None
        maior_area = 0

        for cnt in contornos:
            area = cv2.contourArea(cnt)
            if area > maior_area:
                maior_contorno = cnt
                maior_area = area
        
        if (maior_area > 1000) and  not self.STATUS['searchCreepMistaked']:
            self.STATUS['searchCreepDetected'] = True  #! Faz o robô centralizar E aproximar do creeper
            self.STATUS['trilhaON'] = False #! Faz o robô parar de seguir linha
            #$ Ativa CHECKPOINT p/ salvar direção de giro
            self.STATUS['checkpoint'] = True

            print(f"Creeper avistado -- {maior_area}")
            print(f"§=> trilha: {self.STATUS['trilhaON']}")


        if not maior_contorno is None:
            media = maior_contorno.mean(axis=0)
            media = media.astype(np.int32)
            # Desenha um circulo no centro do creeper
            # cv2.circle(self.output_img, (media[0], media[1]), 5, [100, 255, 100])

            # Guarda a posição do centro em ALVOS:
            self.ALVO['centro'] = media[0] #(x,y)
 

        #! DESENHA O CENTRO DA ÁREA NA IMG ORIGINAL

    #@ chamada qnd searchCreepDetected = True
    def centraliza_robo(self, alvo):
        # Guarda a posição inicial do alvo, para retornar para o sentido contrário depois
        if (alvo[0] > self.CENTRO_ROBO[0]) and self.STATUS['checkpoint']: 
            # Alvo à direita do robô --> Quer dizer que o robô girará à direita
            self.ALVO['sentidoGiro'] = 'direita'
            self.STATUS['checkpoint'] = False
            print("Registrando direção de giro -- DIREITA")

        elif (alvo[0] < self.CENTRO_ROBO[0]) and self.STATUS['checkpoint']:
            self.ALVO['sentidoGiro'] = 'esquerda'
            self.STATUS['checkpoint'] = False
            print("Registrando direção de giro -- ESQUERDA")


        # Centraliza o robô, apontando para o creeper
        if (abs(alvo[0] - self.CENTRO_ROBO[0]) >= 20):
            print("Centralizar robô com o Alvo")
            if (alvo[0] > self.CENTRO_ROBO[0]):
                # Gira p/ direita
                vel = Twist(Vector3(0.0,0,0), Vector3(0,0,-0.1))
                self.velocidade_saida.publish(vel)
            else:
                # Gira p/ esquerda 
                vel = Twist(Vector3(0.0,0,0), Vector3(0,0,0.1))
                self.velocidade_saida.publish(vel)


        else:
            vel = Twist(Vector3(0.0,0,0), Vector3(0,0,0))
            self.velocidade_saida.publish(vel)
            print("Centralizado")
            print("Ativar ARUCO!")
            print(self.ALVO['sentidoGiro'])
            self.STATUS['arucoON'] = True


            #* O robô ativa o Aruco e tenta ler o id
            #* Fazer aproximar, caso a leitura do id não seja precisa
            #* Quando conseguir ler com precisão, decidir oq fazer (caso id Correto e caso id Errado)
            #* Caso id errado: Fazer girar no sentido oposto ao salvo em self.ALVO['direcaoGiro']
            #* Caso id correto: 1) Avançar e pegar ou 2) Guardar posição para voltar depois

        #TODO: Fazer o robô voltar para a direção anterior, caso NÃO seja o id do creeper

    def retorna_para_trilha(self):
       
        if self.ALVO['sentidoGiro'] == 'esquerda':
            self.gira_direita()
            self.STATUS['searchTrilha'] = True 
            self.STATUS['searchCreepMistaked'] = False
            #! Ativa a função que procura a trilha e faz o robô SEGUIR qnd identificar

        elif self.ALVO['sentidoGiro'] == 'direita':
            self.gira_esquerda()
            self.STATUS['searchTrilha'] = True 
            self.STATUS['searchCreepMistaked'] = False
            #! Ativa a função que procura a trilha e faz o robô SEGUIR qnd identificar

        else:
            self.vel_tras()
            self.STATUS['searchTrilha'] = True 
            self.STATUS['searchCreepMistaked'] = False
            #! Ativa a função que procura a trilha e faz o robô SEGUIR qnd identificar


    #@ searchTrilha = True
    def encontra_contornos(self, mask):
        #<> FAZER ESSA FUNÇÃO SER "IDENTIFICA TRILHA" 
        #<> --> trabalhar com ÁREA ou nº de contornos
        #$ Retorna um conjunto de contornos
        contornos, arvore = cv2.findContours(mask.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        if len(contornos) > 0:
            self.STATUS["trilhaON"] = True
            self.STATUS["searchTrilha"] = False
        else:
            self.STATUS["trilhaON"] = False
            self.STATUS["searchTrilha"] = True

        return contornos

    def encontra_centro_contornos(self, contornos):
        #$ Retorna tupla que é a mediana dos centros
        X = []
        Y = []

        # Determina o centro dos contornos amarelos
        for contorno in contornos:
            M = cv2.moments(contorno)

            # Usando a expressão do centróide
            if M["m00"] == 0: 
                M["m00"] = 1
            cX = int(M["m10"] / M["m00"])
            cY = int(M["m01"] / M["m00"])
            # Exclui pontos mais à esquerda e da parte central superior
            if (cX > WIDTH//4 and cX < WIDTH//4 * 3 and cY > 350) or (cX > WIDTH//4 * 3 and cY > 305):
                X.append(cX)
                Y.append(cY)
        mediana_X = int(np.median(X))
        mediana_Y = int(np.median(Y))
        centro = (mediana_X, mediana_Y)

        return centro

    def percorre_trilha(self, frame):
        #<> FAZER RECEBER CENTRO apenas E CONTROLAR DIREÇÃO DE MOVIMENTO
        #! Controla processo de manter o robô andando na pista
        # Segmenta amarelo, acha contornos e obtém o centro
        mask_amarelo = self.segmenta_cor(frame.copy(), "amarelo")
        contornos = self.encontra_contornos(mask_amarelo)
        try:
            centro = self.encontra_centro_contornos(contornos)
        except:
            centro = self.CENTRO_ROBO
        # Controla direção do robô
        if centro[0] - self.CENTRO_ROBO[0] > self.tolerancia_giro:
            self.vel_direita()
            print("Trilha -- DIREITA")
        elif centro[0] - self.CENTRO_ROBO[0] < -self.tolerancia_giro:
            self.vel_esquerda()
            print("Trilha -- ESQUERDA")
        else:
            self.vel_frente()
            print("Trilha -- FRENTE")
        
        #! ======================= TESTE ===========================
        self.draw_crossHair(self.output_img, centro, 5, [0, 255, 0])
        self.draw_crossHair(self.output_img, self.CENTRO_ROBO, 5, [0, 0, 255])
        cv2.imshow("TESTE", self.output_img)
        #! =========================================================

    def vel_parado(self):
        vel = Twist(Vector3(0, 0, 0), Vector3(0, 0, 0))
        self.velocidade_saida.publish(vel)

    def vel_frente(self):
        vel = Twist(Vector3(0.2, 0, 0), Vector3(0, 0, 0))
        self.velocidade_saida.publish(vel)

    def vel_tras(self):
        vel = Twist(Vector3(-0.2, 0, 0), Vector3(0, 0, 0))
        self.velocidade_saida.publish(vel)

    def vel_direita(self):
        vel = Twist(Vector3(0.1, 0, 0), Vector3(0, 0, -0.1))
        self.velocidade_saida.publish(vel)

    def vel_esquerda(self):
        vel = Twist(Vector3(0.1, 0, 0), Vector3(0, 0, 0.1))
        self.velocidade_saida.publish(vel)

    def gira_direita(self):
        vel = Twist(Vector3(0, 0, 0), Vector3(0, 0, -0.2))
        self.velocidade_saida.publish(vel)

    def gira_esquerda(self):
        vel = Twist(Vector3(0, 0, 0), Vector3(0, 0, 0.2))
        self.velocidade_saida.publish(vel)
