#!/usr/bin/python
# -*- coding: utf-8 -*-

import cv2
import numpy as np
import math
import cv2
import cv2.aruco as aruco 
import biblioteca as bibliot
import cormodule as cormod

def procesa_imagem_pista(bgr):
    """
	Função que processa uma imagem, buscando a pista.
    Retorna:
        centro_pista: coordenadas do centro de massa da linha 
        amarela, que sao as coordenadas do centro da pista

        bgr_copy: copia da imagem da camera do robo, com
        retangulos desenhados por cima, que servem como antolhos,
        para o robo. (Usado para fins de teste)

        mask: mascara das linhas amarelas.
	""" 

    bgr_copy = bgr.copy()
    # image = cv2.rectangle(image, start_point, end_point, color, thickness)
    bgr_copy = cv2.rectangle(bgr_copy, (0,0), ((bgr.shape[1]//2)-120, bgr.shape[0]) , (0,0,255), -1)
    bgr_copy = cv2.rectangle(bgr_copy, ((bgr.shape[1]//2) + 175,0), (bgr.shape[1], bgr.shape[0]) , (0,0,255), -1)

    mask = bibliot.segmenta_linha_amarela(bgr_copy)
    centro_pista = bibliot.center_of_mass(mask)

    

    return centro_pista, bgr_copy, mask

def aruco_read(img_copy):
    """
	Função que processa uma imagem, buscando os QR codes.
	Retorna:
		img_copy: copia da imagem da camera do robo, com
        o contorno dos QRcodes desenhados por cima, e com os IDs dos 
        QRcodes sobrescritos na imagem.

        ids: lista dos IDs dos QRcodes, que a funcao esta
        detectando na imagem.
	""" 

    aruco_dict  = aruco.getPredefinedDictionary(aruco.DICT_6X6_250)
    # parameters  = aruco.DetectorParameters_create()
    # parameters.minDistanceToBorder = 0
    # parameters.adaptiveThreshWinSizeMax = 1000
    gray = cv2.cvtColor(img_copy, cv2.COLOR_BGR2GRAY)
    corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict) #, parameters=parameters)
    # if ids is None:
    #     return None
    # for i in range(len(ids)):
    #     print('ID: {}'.format(ids[i]))
        
    #     for c in corners[i]: 
    #         for canto in c:
    #             print("Corner {}".format(canto))
    aruco.drawDetectedMarkers(img_copy, corners, ids)

    return img_copy, ids
    