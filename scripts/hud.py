import rospy 
import numpy as np 
import cv2 as cv2

WHITE = (255, 255, 255)
GRAY = (128, 128, 128)
LARANJA = (0, 180, 200)
DARK_BLUE = (180, 0, 0)
DARK_RED = (0, 0, 180)
PURPLE = (200, 0, 150)

def drawText(output_img, texto, posicao, color=(255,255,255), font=cv2.FONT_HERSHEY_SIMPLEX, width=1, size=0.4 ):
    cv2.putText(output_img, texto, posicao, font, size, color, width, cv2.LINE_AA)


def draw_crossHair(output_img, point, size, color):
    # Desenha + no meio da tela
    x,y = point
    cv2.line(output_img,(x - size,y),(x + size,y),color, thickness=2)
    cv2.line(output_img,(x,y - size),(x, y + size),color, thickness=2)

def drawHUD(output_img, status, infos):
    CENTER_X = output_img.shape[1]//2
    CENTER_Y = output_img.shape[0]//2

    # DESENHA CROSSHAIR NO CENTRO e TOLERANCIA EM X:
    if infos['cm_trilha'] is not None:
        delta_x = infos['cm_trilha'][0] - CENTER_X
        if delta_x > 0:
            cv2.line(output_img, (CENTER_X, CENTER_Y), (CENTER_X + delta_x, CENTER_Y), color=DARK_BLUE, thickness=2)
        
        else:
            cv2.line(output_img, (CENTER_X, CENTER_Y), (CENTER_X + delta_x, CENTER_Y), color=DARK_RED, thickness=2)
    
    draw_crossHair(output_img, (CENTER_X, CENTER_Y), size=10, color=LARANJA)
    
    # PRINTA OS STATUS
    drawText(output_img, "STATUS: ", (470, 16), size=0.5)
    contagem = 1
    for estado, valor in status.items():
        if valor:
            color = WHITE
        else:
            color = GRAY

        drawText(output_img, estado, posicao=(490, 16 + contagem*16), color=color)
        contagem += 1
            
    # PRINTA A VELOCIDADE (v e w)
    if infos['v'] is not None and infos['w'] is not None:
        drawText(output_img, f"v= {infos['v']:.3f} w= {infos['w']:.3f}", posicao=(400, 460), size=0.6, width=2)

    # DESENHA UM CIRCULO NO CENTRO DO CREEPER:
    if infos['centro_creeper'] is not None and status['searchCreepON']:
        cv2.circle(output_img, (infos['centro_creeper'][0], infos['centro_creeper'][1]), radius=5, color=PURPLE, thickness=-1)


    # DESENHA LINHA DE REGRESSÃO
    if infos['regressao'] is not None and status['trilhaON']:
        ximg, yimg, coef_angular, coef_linear = infos['regressao']

        y_min , y_max = np.min(yimg), np.max(yimg)

        x_min = int(coef_angular * y_min + coef_linear)
        x_max = int(coef_angular * y_max + coef_linear)

        cv2.line(output_img, (x_min, y_min), (x_max, y_max), color=DARK_BLUE, thickness=2)

    # DESENHA BASE DETECTADA
    if infos['base_detectada'] is not None and status['searchBaseON']:
        p1 = infos['base_detectada'][2]
        p2 = infos['base_detectada'][3]
        label = infos['base_detectada'][0]
        accuracy = infos['base_detectada'][1]
        cv2.rectangle(output_img, p1, p2, DARK_BLUE, 1)
        drawText(output_img, f"{label} -- {accuracy:.2f}%", (p1[0], p1[1]-5), color=DARK_RED)

