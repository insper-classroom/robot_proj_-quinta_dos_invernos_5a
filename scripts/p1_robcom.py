#! /usr/bin/env python3
# -*- coding:utf-8 -*-
from __future__ import print_function, division
import rospy
import numpy as np
import cv2
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, CompressedImage, LaserScan
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import Twist, Vector3, Pose, Vector3Stamped
import aruco1
import mobilenet_simples
import hud
import statsmodels.api as sm
from math import atan, pi

"""
Arquivo para trabalharmos usando classe e orientação a objetos:
"""
goal = ("green", 11, "cow")

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
        self.imagem = rospy.Subscriber(self.topico_imagem, CompressedImage, self.trata_frame, queue_size=1, buff_size=2**24)
        self.scan = rospy.Subscriber("/scan", LaserScan, self.trataScan)
        self.velocidade_saida = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        
        self.ombro = rospy.Publisher("/joint1_position_controller/command", Float64, queue_size=1)
        self.garra = rospy.Publisher("/joint2_position_controller/command", Float64, queue_size=1)


        self.output_img = None
        self.CENTRO_ROBO = None
        self.distancias = None
        self.angulo = 0


        self.STATUS = {
            'trilhaON': True,                   # faz o robô percorrer a trilha
            'searchTrilha': False,              # faz o robô girar à direita até achar uma trilha
            
            'arucoON': False,                   # ativa o leitor do aruco
            'searchCreepON': True,              # ativa a detecção do creeper em função da cor
                'confirmId': False,             # detectou massa de creeper na imagem --> 1) Deixa de Seguir Trilha, 2) Centraliza, 3) investiga se é do id desejado
                'searchCreepMistaked': False,       # confirmId detectou q não é o id correto: --> volta para trilha
                    #! limpar dados em self.ALVO

                'searchCreepConfirmed': False,      # confirmId detectou q É o id correto: --> pega o Creeper
                    'garraPosicionada': False,      #* Levanta o ombro parcialmente e abre a garra
                    'aproximarCreep': False,        #* Aproxima do creep se alinhando com ele 
                    'creepProximo': False,          #* aproxima até d<18

                    'creepCapturado': False,            #* fecha garra e levante o ombro
            'retornarTrilha': False,
            'searchBaseON': False,               # ativa o mobileNet para identificar a base
                'searchBaseConfirmed': False,   # faz o robô centralizar com a base identificada
                'aproximaBase': False,          # faz o robô se aproximar da base, centralizando-se
                'deployCreep': False,           # faz o robô dar deploy do creeper
                'creepDeployed': False,         # faz o robô se afastar da base e procurar a Trilha novamente
            'checkpoint': False,                # variável utilizada para gravar informações pontualmente (sem repetições)
            'delayReturn': False,
        }

        self.ALVO = {
            'centro': None,         # guarda o centro do Alvo (base ou creeper)
            'sentidoGiro': None,    # guarda o sentido inicial do giro para centralizar no alvo
        }

        self.CLOCK = {
            'to': None,
            'tf': None,
        }

        self.INFOS = {
            'v': None,
            'w': None,
            'cm_trilha': None,
            'centro_creeper': None,
            'base_detectada': None,
            'regressao': None,
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

            if self.STATUS["searchTrilha"]:

                if self.STATUS["retornarTrilha"]:
                    self.retorna_para_trilha()

                else:
                    self.gira_direita()

            if self.STATUS['confirmId']:
                #$ if confirmId:
                # --> Faz o robô confirmar se o creeper avistado é da id desejada
                # --> Se achar: (1) - Marca posição; (2) - Vai até ele e pega
                self.centraliza_robo(self.ALVO['centro'])
            
            if self.STATUS['searchCreepMistaked'] and not self.STATUS['trilhaON']:
                self.STATUS['arucoON'] = False
                self.STATUS['searchCreepON'] = False
                self.STATUS['retornarTrilha'] = True
                self.retorna_para_trilha() # faz girar no sentido contrário ao self.ALVO['sentidoGiro']

                #! IDENTIFICAR TRILHA 
                #! --> qnd identificado --> para de girar --> ativa TRILHA ON e desativa MISTAKED
                #! DESATIVAR o searchCreepON e ativá-lo após alguns segundos

            if self.STATUS['searchCreepMistaked'] and self.STATUS['trilhaON']:
                #! Robô acabou de voltar a seguir a pista...
                self.STATUS['arucoON'] = False
                self.CLOCK['tf'] = rospy.get_time()
                delta_t = self.CLOCK['tf'] - self.CLOCK['to']
                print(f"delta_t = {delta_t}")
                if delta_t > 6:
                    self.STATUS['searchCreepMistaked'] = False
                    self.STATUS['searchCreepON'] = True

            if self.STATUS['searchCreepConfirmed'] and not self.STATUS['creepCapturado']:
                self.STATUS['arucoON'] = False
                self.CLOCK['tf'] = rospy.get_time()
                delta_t = self.CLOCK['tf'] - self.CLOCK['to']
                print(f"delta_t = {delta_t:.4f}")

                # Espera 1 segundo parado, para posicionar a garra e avançar
                if delta_t > 1 and not self.STATUS['garraPosicionada']:
                    self.ombro.publish(-0.4)    # levanta o ombro parcialmente
                    self.garra.publish(-1)      # abre a garra
                    self.STATUS['garraPosicionada'] = True
                    #! Checar se ele faz os dois ao mesmo tempo

                if self.STATUS['garraPosicionada'] and not self.STATUS['creepProximo']: #* previne q o robô ande p/ frente sem desejarmos
                    print(self.distancias[358], self.distancias[0], self.distancias[2])


                    if self.distancias[0] > 0.19: # and self.distancias[358] > 0.19 and self.distancias[2] > 0.19: 
                        #<> PRECISA SER REFINADO!!!!! 
                        self.aproxima_alvo_centralizando(self.ALVO['centro'])
                    else:                    
                        self.STATUS['creepProximo'] = True
                        self.vel_parado()
                        self.CLOCK['to'] = rospy.get_time()

                if self.STATUS['creepProximo']:
                    # Fechar pinça
                    self.garra.publish(0)    # fecha garra 
                    self.CLOCK['tf'] = rospy.get_time()
                    delta_t = self.CLOCK['tf'] - self.CLOCK['to']
                    if delta_t > 1:
                        # Levantar ombro c/ o creep
                        self.ombro.publish(1.5)  
                        self.STATUS['creepCapturado'] = True
                        print("CREEP CAPTURADO COM SUCESSO!")
                        self.CLOCK['to'] = rospy.get_time()


            if self.STATUS['creepCapturado']:
                self.STATUS['searchCreepON'] = False
                self.STATUS['garraPosicionada'] = False 
                self.STATUS['creepProximo'] = False
                self.STATUS['alinhamentoOK'] = False

                self.CLOCK['tf'] = rospy.get_time()
                delta_t = self.CLOCK['tf'] - self.CLOCK['to']
                if delta_t < 1:
                    self.STATUS['retornarTrilha'] = True
                    self.STATUS['searchTrilha'] = True

                else:
                    self.STATUS['searchBaseON'] = True
                    self.STATUS['searchCreepConfirmed'] = False 
                    self.STATUS['creepCapturado'] = False
                    
            #@ searchBaseON (trataFrame)

            if self.STATUS['searchBaseConfirmed']:
                # deixa de seguir trilha e centraliza com o alvo
                self.STATUS['trilhaON'] = False
                self.centraliza_robo(self.ALVO['centro'])

            if self.STATUS['aproximaBase']:
                self.STATUS['searchBaseConfirmed'] = False

                if self.distancias[0] >= 0.4:
                    self.aproxima_alvo_centralizando(self.ALVO['centro'])
                    # <> CHECAR SE A MOBILENET funciona EM TODA APROXIMACAO
                else:
                    # robô próximo da base --> fica parado
                    self.vel_parado()
                    self.STATUS['searchBaseON'] = False
                    self.STATUS['aproximaBase'] = False
                    self.STATUS['deployCreep'] = True 
                    self.CLOCK['to'] = rospy.get_time()
                    print("Robô próximo da Base -- Deploy Creeper!!")
                    #! ativar proximo status: 1. abaixar ombro 2. abrir garra 

            if self.STATUS['deployCreep']:
                self.CLOCK['tf'] = rospy.get_time()
                delta_t = self.CLOCK['tf'] - self.CLOCK['to']
                print(f"delta_t = {delta_t}")
                if delta_t > 0.5 and delta_t < 1.0:
                    self.ombro.publish(-0.4)

                elif delta_t > 1.5 and delta_t < 2.0:
                    self.garra.publish(-1)

                elif delta_t > 2.5:
                    self.ombro.publish(-1.0)
                    self.STATUS['creepDeployed'] = True
                    self.STATUS['searchTrilha'] = True
                    self.STATUS['retornarTrilha'] = True
                    self.STATUS['deployCreep'] = False
                    self.CLOCK['to'] = rospy.get_time()

            if self.STATUS['creepDeployed']:
                self.retorna_para_trilha()

                
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

            #! ======================= TESTE ===========================
            hud.drawHUD(self.output_img, self.STATUS, self.INFOS)
            cv2.imshow("TESTE", self.output_img)
            cv2.waitKey(1)
            #! =========================================================

            HEIGHT = cv_image_original.shape[0]
            WIDTH = cv_image_original.shape[1]

            # self.searchTrilha(cv_image_original)

            if self.STATUS['trilhaON']:
                #$ Segmenta Amarelo (Trilha) e faz robô andar a trilha
                self.percorre_trilha(self.output_img)
                cv2.waitKey(1)

            if self.STATUS["searchTrilha"]:
                #$ Permanece girando até que encontre a pista
                self.encontra_contornos(self.segmenta_cor(self.output_img, 'yellow'))

            if self.STATUS['searchCreepON']:
                #$ Segmenta Creeper
                #>> apenas para visualização (não é necessário exibir a imagem toda vez)
                """
                Para que a imagem do Creeper não atrapalhe na detecção da linha, 
                acredito que tenhamos que analisar os filtros em imagens diferentes.  
                """
                segmentado_creeper = self.segmenta_cor(cv_image_original, goal[0])  #! Depois, vamos automatizar para escolher a cor da missão
                self.calcula_area(segmentado_creeper)  # > 800
                cv2.imshow("seg_creeper", segmentado_creeper)
                cv2.waitKey(1)

                #! CALCULA ÁREA --> se maior q X -> confirmId =True --> centraliza no creeper e aproxima.        
            
            if self.STATUS['arucoON']:
                #! configurar um deltaT para chamar o localiza_id
                #! CONFIGURAR STATUS p/ IDENTIFICAR ID
                self.localiza_id(frame, self.ALVO['centro'])
                self.CLOCK['to'] = rospy.get_time()
                self.STATUS['confirmId'] = False 

            if self.STATUS['searchBaseON']:
                self.localiza_base(frame)

        except CvBridgeError as e:
            print('ex', e)  

    def trataScan(self, scanner):
        """ Guarda as distâncias em self.distancias """
        self.distancias = np.array(scanner.ranges).round(decimals=2)  

    def segmenta_cor(self, frame, COR):
        output = frame.copy()
        if COR == 'yellow':
            bgr_min = np.array([0, 190, 190], dtype=np.uint8)
            bgr_max = np.array([50, 255, 255], dtype=np.uint8)
            segmentado_cor = cv2.inRange(output, bgr_min, bgr_max)

        elif COR == 'green':
            bgr_min = np.array([0, 50, 0], dtype=np.uint8)
            bgr_max = np.array([5,255,5], dtype=np.uint8)
            segmentado_cor = cv2.inRange(output, bgr_min, bgr_max)

        elif COR == 'blue':
            # Azul(Malibu)-HSV 360: [191,100,69] --> 180: [95, 50, 35]
            #! Tratar como HSV:
            output = cv2.cvtColor(output, cv2.COLOR_BGR2HSV)
            hsv_min = np.array([93, 40, 28], dtype=np.uint8)
            hsv_max = np.array([98, 255,255], dtype=np.uint8)
            segmentado_cor = cv2.inRange(output, hsv_min, hsv_max)

        elif COR == 'orange':
            bgr_min = np.array([0, 0, 180], dtype=np.uint8)
            bgr_max = np.array([5,25,255], dtype=np.uint8)
            segmentado_cor = cv2.inRange(output, bgr_min, bgr_max)

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
        print(f"AREA -- {maior_area} ")
        if (maior_area > 800) and  not self.STATUS['searchCreepMistaked'] and not self.STATUS['searchCreepConfirmed']:
            #<> Fazer acontecer SÓ SE NENHUM searchCreep (Mistaked/Confirmed) tiver ativado:
            self.STATUS['confirmId'] = True  #! Faz o robô centralizar E aproximar do creeper
            self.STATUS['trilhaON'] = False #! Faz o robô parar de seguir linha
            #$ Ativa CHECKPOINT p/ salvar direção de giro
            self.STATUS['checkpoint'] = True

            print(f"Creeper avistado -- {maior_area}")
            print(f"§=> trilha: {self.STATUS['trilhaON']}")


        if not maior_contorno is None:
            media = maior_contorno.mean(axis=0)
            media = media.astype(np.int32)

            # Guarda a posição do centro em ALVOS:
            self.ALVO['centro'] = media[0] #(x,y)
            self.INFOS['centro_creeper'] = media[0]
 

    def centraliza_robo(self, alvo):
       
        #@ chamada1: qnd confirmId = True --> é desativado após confirmar / refutar
        #@ chamada2: qnd searchBaseConfirmed = True --> é desativado qnd começa a aproximar da Base
        if self.STATUS['checkpoint']:
            # Guarda a posição inicial do alvo, para retornar para o sentido contrário depois
            if (alvo[0] > self.CENTRO_ROBO[0]): 
                # Alvo à direita do robô --> Quer dizer que o robô girará à direita
                self.ALVO['sentidoGiro'] = 'direita'
                self.STATUS['checkpoint'] = False
                print("Registrando direção de giro -- DIREITA")

            elif (alvo[0] < self.CENTRO_ROBO[0]):
                self.ALVO['sentidoGiro'] = 'esquerda'
                self.STATUS['checkpoint'] = False
                print("Registrando direção de giro -- ESQUERDA")

        # Centraliza o robô, apontando para o creeper
        if (abs(alvo[0] - self.CENTRO_ROBO[0]) >= 20):
            # Se for antes de confirmar o id, apenas rotaciona
            if not self.STATUS['searchCreepConfirmed']:
                # print("Centralizar robô com o Alvo")
                if (alvo[0] > self.CENTRO_ROBO[0]):
                    # Gira p/ direita
                    print("    Centralizando robô com o alvo -> ")
                    vel = Twist(Vector3(0.0,0,0), Vector3(0,0,-0.2))
                    self.velocidade_saida.publish(vel)
                else:
                    # Gira p/ esquerda 
                    print(" <- Centralizando robô com o alvo    ")
                    vel = Twist(Vector3(0.0,0,0), Vector3(0,0,0.2))
                    self.velocidade_saida.publish(vel)

        else:
            vel = Twist(Vector3(0.0,0,0), Vector3(0,0,0))
            self.velocidade_saida.publish(vel)
            print(" -- Centralizado! -- ")
            if self.STATUS['confirmId']:
                print("Ativar ARUCO!")
                self.STATUS['arucoON'] = True

            elif self.STATUS['searchBaseConfirmed']:
                self.STATUS['searchBaseConfirmed'] = False
                self.STATUS['aproximaBase'] = True

            #* O robô ativa o Aruco e tenta ler o id
            #* Fazer aproximar, caso a leitura do id não seja precisa
            #* Quando conseguir ler com precisão, decidir oq fazer (caso id Correto e caso id Errado)
            #* Caso id errado: Fazer girar no sentido oposto ao salvo em self.ALVO['direcaoGiro']
            #* Caso id correto: 1) Avançar e pegar ou 2) Guardar posição para voltar depois

        #TODO: Fazer o robô voltar para a direção anterior, caso NÃO seja o id do creeper

    def aproxima_alvo_centralizando(self, alvo):
        if (abs(alvo[0] - self.CENTRO_ROBO[0]) >= 20):
                # Rotaciona E Avança
                if (alvo[0] > self.CENTRO_ROBO[0]):
                    # Gira p/ direita
                    print("\t    \t d:", self.distancias[0], "\t -->")
                    vel = Twist(Vector3(0.15,0,0), Vector3(0,0,-0.1))
                    self.velocidade_saida.publish(vel)
                else:
                    # Gira p/ esquerda 
                    print("\t <-- \t d:", self.distancias[0])
                    vel = Twist(Vector3(0.15,0,0), Vector3(0,0,0.1))
                    self.velocidade_saida.publish(vel)

        else:
            print("d:", self.distancias[0],  "\t  ==")
            vel = Twist(Vector3(0.2,0,0), Vector3(0,0,0.1))
            self.velocidade_saida.publish(vel)
    
    def retorna_para_trilha(self):
        self.STATUS['arucoON'] = False
        print ("Retornar para pista ")
        self.STATUS['searchTrilha'] = True  #! Ativa função encontro_contornos

        self.CLOCK['tf'] = rospy.get_time()
        delta_t = self.CLOCK['tf'] - self.CLOCK['to']

        if self.STATUS['creepDeployed'] and delta_t < 1.0:
            self.vel_tras()
            self.garra.publish(0)
            # self.STATUS['searchCreepMistaked'] = False
            #! Ativa a função que procura a trilha e faz o robô SEGUIR qnd identificar

        elif self.ALVO['sentidoGiro'] == 'esquerda':
            self.gira_direita()
            # self.STATUS['searchCreepMistaked'] = False
            #! Ativa a função que procura a trilha e faz o robô SEGUIR qnd identificar

        elif self.ALVO['sentidoGiro'] == 'direita':
            self.gira_esquerda()
            # self.STATUS['searchCreepMistaked'] = False
            #! Ativa a função que procura a trilha e faz o robô SEGUIR qnd identificar

    def encontra_contornos(self, mask):
        #@ searchTrilha = True
        #<> FAZER ESSA FUNÇÃO SER "IDENTIFICA TRILHA" 
        #<> --> trabalhar com ÁREA ou nº de contornos
        #$ Retorna um conjunto de contornos
        contornos, arvore = cv2.findContours(mask.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        if len(contornos) > 0:
            if self.STATUS['retornarTrilha']:
                self.STATUS['delayReturn'] = True
                self.CLOCK['to2'] = rospy.get_time()

            
            self.STATUS["searchTrilha"] = False
            self.STATUS['retornarTrilha'] = False
            self.STATUS["trilhaON"] = True
            self.STATUS['creepDeployed'] = False
        else:
            self.STATUS["trilhaON"] = False
            self.STATUS["searchTrilha"] = True

        return contornos

    def encontra_centro_contornos(self, contornos):
        #$ Retorna tupla que é a mediana dos centros
        X = []
        Y = []
        selecionados = [] # guarda contornos que atendem à condição desejada

        if self.STATUS['delayReturn']:
            self.CLOCK['tf2'] = rospy.get_time()
            delta_t = self.CLOCK['tf2'] - self.CLOCK['to2']
            if delta_t > 3:
                self.STATUS['delayReturn'] = False
        # Determina o centro dos contornos amarelos
        for contorno in contornos:
            M = cv2.moments(contorno)

            # Usando a expressão do centróide
            if M["m00"] == 0: 
                M["m00"] = 1
            cX = int(M["m10"] / M["m00"])
            cY = int(M["m01"] / M["m00"])
            # Exclui pontos mais à esquerda e da parte central superior
            if (cX > WIDTH//4 and cX < WIDTH//4 * 3 and cY > 350) or (cX > WIDTH//4 * 3 and cY > 320) or self.STATUS['delayReturn']:
                X.append(cX)
                Y.append(cY)
                selecionados.append(contorno)
        mediana_X = int(np.median(X))
        mediana_Y = int(np.median(Y))
        centro = (mediana_X, mediana_Y)
        self.INFOS['cm_trilha'] = centro

        return (centro, selecionados)

    def percorre_trilha(self, frame):
        #<> FAZER RECEBER CENTRO apenas E CONTROLAR DIREÇÃO DE MOVIMENTO
        #! Controla processo de manter o robô andando na pista
        # Segmenta amarelo, acha contornos e obtém o centro
        mask_amarelo = self.segmenta_cor(frame.copy(), "yellow")
        contornos = self.encontra_contornos(mask_amarelo)
        try:
            self.processa_ajuste_linear(mask_amarelo)
        except:
            pass
        try:
            centro, selecionados = self.encontra_centro_contornos(contornos)
            
            # Ajusta velocidade linear com controle derivativo
            v = (0.25/90)*(90 - abs(self.angulo)) + 0.13

            # Velocidade angular com controle proporcional e derivativo
            erro = centro[0] - self.CENTRO_ROBO[0]
            w = -(0.5/(WIDTH/2))*erro + 0.1*self.angulo/90
            
            vel = Twist(Vector3(v, 0, 0), Vector3(0, 0, w))
            self.velocidade_saida.publish(vel)

            self.INFOS['v'] = v
            self.INFOS['w'] = w
            # for c in selecionados:
            #     cv2.drawContours(frame, [c], -1, [255, 0, 0], 3)
            # self.draw_crossHair(frame, (centro[0], HEIGHT//2), 5, (0,255,0))
            # cv2.line(frame, (WIDTH//2, 0), (WIDTH//2, HEIGHT), color=(0,0,255), thickness=5)
            # font = cv2.FONT_HERSHEY_SIMPLEX
            # cv2.putText(frame, f'Angulo = {self.angulo:.2f} graus',(0,50), font, 1,(255,255,255),2,cv2.LINE_AA)
            # cv2.imshow("TESTE", frame)

        except:
            print("Trilha perdida")
            self.STATUS["trilhaON"] = False
            self.STATUS["searchTrilha"] = True

    def localiza_id(self, frame, centro_alvo):

        ids = aruco1.roda_todo_frame(frame, centro_alvo)
        try:
            if ids is not None:
                print(f"id = {ids[0][0]}")
            
                if ids[0][0] == goal[1]:
                    self.STATUS['searchCreepConfirmed'] = True


                # Se for falso, volta para a pista:
                else:
                    print('não achou')
                    self.STATUS['searchCreepMistaked'] = True

            else:
                print("Aruco não identificado!")
                self.STATUS['searchCreepMistaked'] = True
        except CvBridgeError as e:
            print('ex', e)

    def localiza_base(self, frame):
        #@ chamado qnd searchBaseON = True --> desativa somente qnd próx. da base
        imagem, results = mobilenet_simples.detect(frame)
        thresholds = {"dog": 95, "horse": 90, "car": 90, "cow": 70}

        try:
            if len(results) > 0:
                self.INFOS['base_detectada'] = results[0]
                if results[0][0] == goal[2]:
                    print(f"{results[0][0]}: -- {results[0][1]}")
            
                if results[0][0] == goal[2] and results[0][1] >= thresholds[goal[2]]:
                    self.ALVO['centro'] = (((results[0][2][0] + results[0][3][0])//2), ((results[0][2][1] + results[0][3][1])//2))
                    
                    if not self.STATUS['searchBaseConfirmed']: 
                        # ativa o Checkpoint apenas uma vez, qnd chamar centraliza_robo.
                        self.STATUS['checkpoint'] = True
                    
                    if not self.STATUS['aproximaBase']: # evita que o searchBaseConfirmed seja reativado
                        self.STATUS['searchBaseConfirmed'] = True
                        print(f'Base [{results[0][0]}] detectada!')

                # Se for falso, volta para a pista:
                else:
                    print('Procurando Base...')

        except CvBridgeError as e:
            print('ex', e)

    def regressao_linear(self, mask):
        #$ Recebe imagem limiarizada e retorna os coeficientes
        #$ da reta (linear e angular)
        pontos = np.where(mask==255)
        ximg = []
        yimg = []
        # Filtra pontos amarelos usados no equacionamento
        for i in range(len(pontos[0])):
            if (pontos[1][i] > WIDTH//4 and pontos[1][i] < WIDTH//4 * 3 and pontos[0][i] > 350) or (pontos[1][i] > WIDTH//4 * 3 and pontos[0][i] > 320):
                ximg.append(pontos[1][i])
                yimg.append(pontos[0][i])
        yimg_c = sm.add_constant(yimg)
        # Aplica modelo e obtém coeficientes
        model = sm.OLS(ximg,yimg_c)
        results = model.fit()
        coef_angular = results.params[1]
        coef_linear =  results.params[0]

        self.INFOS['regressao'] = (ximg, yimg, coef_angular, coef_linear)
        return (coef_angular, coef_linear)

    def calcula_angulo(self, coef_angular):
        #! Calcula ângulo que a direção do robô forma com a vertical
        ang_rad = atan(coef_angular)
        self.angulo = 180*ang_rad/pi

    def processa_ajuste_linear(self, mask):
        #$ Atualiza ângulo
        coef_angular, coef_linear = self.regressao_linear(mask)
        self.calcula_angulo(coef_angular)

    def vel_parado(self):
        vel = Twist(Vector3(0, 0, 0), Vector3(0, 0, 0))
        self.velocidade_saida.publish(vel)

    def vel_frente(self):
        vel = Twist(Vector3(0.23, 0, 0), Vector3(0, 0, 0))
        self.velocidade_saida.publish(vel)

    def vel_tras(self):
        vel = Twist(Vector3(-0.2, 0, 0), Vector3(0, 0, 0))
        self.velocidade_saida.publish(vel)

    def vel_direita(self):
        vel = Twist(Vector3(0.2, 0, 0), Vector3(0, 0, -0.25))
        self.velocidade_saida.publish(vel)

    def vel_esquerda(self):
        vel = Twist(Vector3(0.2, 0, 0), Vector3(0, 0, 0.25))
        self.velocidade_saida.publish(vel)

    def gira_direita(self):
        vel = Twist(Vector3(0, 0, 0), Vector3(0, 0, -0.4))
        self.velocidade_saida.publish(vel)

    def gira_esquerda(self):
        vel = Twist(Vector3(0, 0, 0), Vector3(0, 0, 0.4))
        self.velocidade_saida.publish(vel)
