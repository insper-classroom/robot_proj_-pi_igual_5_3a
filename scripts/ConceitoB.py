#! /usr/bin/env python3
# -*- coding:utf-8 -*-

from __future__ import print_function, division
import rospy
import numpy as np
import tf
import math
import cv2
import cv2.aruco as aruco
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, CompressedImage, LaserScan
from cv_bridge import CvBridge, CvBridgeError
from numpy import linalg
from tf import transformations
from tf import TransformerROS
import tf2_ros
from geometry_msgs.msg import Twist, Vector3, Pose, Vector3Stamped
from std_msgs.msg import Float64

from nav_msgs.msg import Odometry
from std_msgs.msg import Header

from biblioteca import *
import cormodule
import ConceitoB_states
import processaImg as procImg

print("EXECUTE ANTES da 1.a vez: ")
print("wget https://github.com/Insper/robot21.1/raw/main/projeto/ros_projeto/scripts/MobileNetSSD_deploy.caffemodel")
print("PARA TER OS PESOS DA REDE NEURAL")

import visao_module


bridge = CvBridge()

cv_image = None
media = []
centro_pista = []
atraso = 1.5E9 # 1 segundo e meio. Em nanossegundos
centro_robo = ()
media_creep = []
maior_area = 0
estado = 1
ranges = []
ids = []

area = 0.0 # Variavel com a area do maior contorno

goal1 = ("blue", 12, "dog")

# Só usar se os relógios ROS da Raspberry e do Linux desktop estiverem sincronizados. 
# Descarta imagens que chegam atrasadas demais
check_delay = False 

resultados = [] # Criacao de uma variavel global para guardar os resultados vistos

x = 0
y = 0
z = 0 
id = 0

frame = "camera_link"
# frame = "head_camera"  # DESCOMENTE para usar com webcam USB via roslaunch tag_tracking usbcam

tfl = 0

tf_buffer = tf2_ros.Buffer()
DIREITA = True

def scaneou(dado):
    """
    Funcao que processa os dados obtidos pelo Laser 
    Scan e os armazena na variavel global ranges.
    Nao retorna nada.
	"""
    global ranges
    ranges = np.array(dado.ranges).round(decimals=2)
    

def roda_todo_frame(imagem):
    """
	Funcao com o objetivo de processar as imagens obtidas pela 
    camera do robo, na seguinte ordem: 
        1.  Processa copia da imagem da camera, usando a funcao procesa_imagem_pista()
            do arquivo processaImg.py. Atribui os valores retornados pela funcao
            as variaveis global centro_pista, local antolho e local mask. Mostra a mask;

        1a. global centro_pista: coordenadas do centro de massa da linha 
            amarela, que sao as coordenadas do centro da pista.
        1b. local antolho: copia da imagem da camera do robo, com
            retangulos desenhados por cima, que servem como antolhos,
            para o robo. (Usado para fins de teste)
        1c. local mask: mascara das linhas amarelas.

        2.  Processa copia da imagem da camera, usando a funcao identifica_cor()
            do arquivo cormodule.py. Atribui os valores retornados pela funcao
            as variaveis global media_creep, global maior_area e local frame;

        2a. global media_creep: coordenadas do centro de massa do creeper,
            que sao as coordenadas do centro do creeper.
        2b. global maior_area: area do contorno do creeper mais proximo.
        2c. local frame: copia da imagem da camera do robo, com o contorno
            do creeper mais proximo desenhado por cima.

        3.  Processa copia da imagem da camera, usando a funcao aruco_read()
            do arquivo processaImg.py(). Atribui os valores retornados pela funcao
            as variaveis local aruco_imshow e global ids. Mostra a aruco_imshow;

        3a. local aruco_imshow: copia da imagem da camera do robo, com
            o contorno dos QRcodes desenhados por cima, e com os IDs dos 
            QRcodes sobrescritos na imagem.
        3b. global ids: lista dos IDs dos QRcodes, que a funcao esta
            detectando na imagem. 

        4.  Calcula as coordenadas do centro da imagem obtida pela camera
            do robo e as armazena na variavel global centro_robo;

        5.  Desenha uma crosshair, com as coordenadas do centro da pista,
            sobre a imagem local frame. Mostra a imagem frame.
	"""
    global cv_image
    global media
    global centro_pista
    global resultados
    global centro_robo
    global media_creep
    global maior_area
    global ids

    try:
        cv_image = bridge.compressed_imgmsg_to_cv2(imagem, "bgr8")

        img_copy = cv_image.copy()
        img_copy_pista = cv_image.copy()
        img_copy_aruco = cv_image.copy()
        img_copy_cor = cv_image.copy()

        centro_pista, antolho, mask = procImg.procesa_imagem_pista(img_copy_pista)
        # cv2.imshow("antolho", antolho)
        cv2.imshow("mask", mask)

        media_creep, maior_area, frame =  cormodule.identifica_cor(img_copy_cor, goal1[0])

        aruco_imshow, ids = procImg.aruco_read(img_copy_aruco)
        cv2.imshow('aruco', aruco_imshow)

        centro_robo = (img_copy.shape[1]//2, img_copy.shape[0]//2)

        if centro_pista is not None:
            crosshair(frame, centro_pista, 4, (0,0,255))

        # cv2.imshow("cv_image", cv_image)
        cv2.imshow("cor", frame)
        cv2.waitKey(1)
    except CvBridgeError as e:
        print('ex', e)

estado2 = False
estado2_trava = False
estado3 = False
estado3_trava = False

def main():
    """
	Funcao com o objetivo de determinar o estado do robo. Realiza a 
    logica de mudanca de estados e executa a funcao do estado atual.
    Nao retorna nada.
	"""
    global ranges
    global estado3
    global estado2
    global estado2_trava
    global estado3_trava

    try:
        middle_sensor_mean = (ranges[355] + ranges[5])/2
        print(middle_sensor_mean, ranges[0])

    except:
        pass

    if ids is not None:
        if maior_area > 1300 and goal1[1] in ids:
            estado2 = True
            try:
                if middle_sensor_mean <= 0.3:
                    estado2_trava = True
                    estado3 = True
    
                    
            except:
                pass

        
    if estado2 == False or (estado2_trava == True and estado3_trava == True):
        maquina_estados[1](velocidade_saida, centro_pista, centro_robo)
    
    if estado2 == True and estado2_trava == False:
        maquina_estados[2](velocidade_saida, media_creep, centro_robo)

    if estado2_trava == True and estado3_trava == False:
        estado3_trava = maquina_estados[3](velocidade_saida, braco_publisher, garra_publisher, media_creep, centro_robo, middle_sensor_mean)


    
    

if __name__=="__main__":
    rospy.init_node("cor")

    topico_imagem = "/camera/image/compressed"

    recebedor = rospy.Subscriber(topico_imagem, CompressedImage, roda_todo_frame, queue_size=4, buff_size = 2**24)
    recebe_scan = rospy.Subscriber("/scan", LaserScan, scaneou)

    print("Usando ", topico_imagem)

    velocidade_saida = rospy.Publisher("/cmd_vel", Twist, queue_size = 1)
    braco_publisher = rospy.Publisher("/joint1_position_controller/command", Float64, queue_size = 1)
    garra_publisher = rospy.Publisher("/joint2_position_controller/command", Float64, queue_size = 1)

    tfl = tf2_ros.TransformListener(tf_buffer) #conversao do sistema de coordenadas 
    tolerancia = 25

    # maquina de estados
    maquina_estados = {
        1: ConceitoB_states.segue_linha,
        2: ConceitoB_states.choca_creep,
        3: ConceitoB_states.pega_creep
    }

    try:
        while not rospy.is_shutdown():
            main()
            rospy.Rate(50).sleep()

    except rospy.ROSInterruptException:
        print("Ocorreu uma exceção com o rospy")
