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
from sensor_msgs.msg import Image, CompressedImage
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
import visao_module
import time

v = 0.1
w = math.pi/20.0

zero = Twist(Vector3(0,0,0), Vector3(0,0,0.0))
frente = Twist(Vector3(v,0,0), Vector3(0,0,0.0))
direita = Twist(Vector3(0,0,0), Vector3(0,0, -w ))
esquerda = Twist(Vector3(0,0,0), Vector3(0,0,w))


def segue_linha(velocidade_saida, centro_pista, centro_robo):
    """
	Funcao com o objetivo de fazer com que o robo siga a linha amarela
    e permaneca na pista. Sempre retorna None e tem como objetivo 
    definir o movimento do robo.
	"""

    limiar = 50
    velocidade_saida.publish(zero)

    if centro_pista is None:
        velocidade_saida.publish(esquerda)
        return None
    
    if len(centro_pista) == 0 or len(centro_robo) == 0:
        velocidade_saida.publish(esquerda)
        return None

    if (centro_pista[0] < centro_robo[0]+limiar) and (centro_pista[0] > centro_robo[0]-limiar):
        velocidade_saida.publish(frente)
            
    elif (centro_pista[0] > centro_robo[0]):
        velocidade_saida.publish(direita)
            
    elif (centro_pista[0] < centro_robo[0]):
        velocidade_saida.publish(esquerda)

    return None

def choca_creep(velocidade_saida, media_creep, centro_robo):
    """
	Funcao com o objetivo de fazer com que o robo se vire em
    direcao ao creeper e se aproxime dele. Sempre retorna None 
    e tem como objetivo definir o movimento do robo.
	"""

    frente_choca = Twist(Vector3(v/2,0,0), Vector3(0,0,0.0))
    direita_choca = Twist(Vector3(0,0,0), Vector3(0,0, -w/2 ))
    esquerda_choca = Twist(Vector3(0,0,0), Vector3(0,0,w/2))

    limiar = 30
    velocidade_saida.publish(zero)

    if media_creep is None:
        velocidade_saida.publish(esquerda_choca)
        return None
    
    if len(media_creep) == 0 or len(centro_robo) == 0:
        velocidade_saida.publish(esquerda_choca)
        return None

    if (media_creep[0] < centro_robo[0]+limiar) and (media_creep[0] > centro_robo[0]-limiar):
        velocidade_saida.publish(frente_choca)
            
    elif (media_creep[0] > centro_robo[0]):
        velocidade_saida.publish(direita_choca)
            
    elif (media_creep[0] < centro_robo[0]):
        velocidade_saida.publish(esquerda_choca)

    return None

def pega_creep(velocidade_saida, braco_publisher, garra_publisher, media_creep, centro_robo, middle_sensor_mean):
    """
	Funcao com o objetivo de fazer com que o robo use a garra e o braco
    para pegar o creeper. Opera na seguinte ordem:
    1. Abre a garra e levanta o braco do robo;
    2. Robo se aproxima, precisamente, do creeper;
    3. Feche a garra do robo, equanto ele segura o creeper,
    e levanta mais o braco.

    Retorna True ou False.

    Retorna False: Equanto nao realiza o fechamento da garra e levantamento do 
    braco, para garantir que o robo permaneca no estado de pegar o creeper. 

    Retorna True: Quando o robo fecha a garra e levanta o creeper, para que ele 
    saia do estado de pegar o creeper e possa voltar ao estado de seguir a pista.
	"""

    frente_devagar = Twist(Vector3(v/4,0,0), Vector3(0,0,0.0))
    direita_devagar = Twist(Vector3(0,0,0), Vector3(0,0, -w/4 ))
    esquerda_devagar = Twist(Vector3(0,0,0), Vector3(0,0,w/4))
    estado_inicial = True

    velocidade_saida.publish(zero)
    limiar = 15
    
    """
    POSICOES DA GARRA:
    Garra recolhida: -1
    Garra 45 graus: -0.5
    Garra para frente: 0
    Garra levantada: 1.5

    Pinça fechada: 0
    Pinça aberta: -1
    """
    if estado_inicial == True:
        pos_braco = Float64()
        pos_braco.data = -0.25
        pos_garra = Float64()
        pos_garra.data = -1

        braco_publisher.publish(pos_braco)
        garra_publisher.publish(pos_garra)
        estado_inicial = False


    if middle_sensor_mean >= 0.18:
        velocidade_saida.publish(zero)

        if media_creep is None:
            velocidade_saida.publish(esquerda_devagar)
            return False
        
        if len(media_creep) == 0 or len(centro_robo) == 0:
            velocidade_saida.publish(esquerda_devagar)
            return False

        if (media_creep[0] < centro_robo[0]+limiar) and (media_creep[0] > centro_robo[0]-limiar):
            velocidade_saida.publish(frente_devagar)
                
        elif (media_creep[0] > centro_robo[0]):
            velocidade_saida.publish(direita_devagar)
                
        elif (media_creep[0] < centro_robo[0]):
            velocidade_saida.publish(esquerda_devagar)

        return False
        
    else:
        pos_garra = Float64()
        pos_garra.data = 0
        pos_braco = Float64()
        pos_braco.data = 1.5

        garra_publisher.publish(pos_garra)
        rospy.sleep(1)
        braco_publisher.publish(pos_braco)
        return True
        

