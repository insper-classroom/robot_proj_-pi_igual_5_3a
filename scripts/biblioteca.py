#!/usr/bin/python
# -*- coding: utf-8 -*-

import cv2
import numpy as np
import math

    
def segmenta_linha_amarela(bgr):
    """
	Função que segmenta a linha amarela, com base em valores hsv.
	""" 
    
    img_bgr = bgr.copy()
    img_hsv = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2HSV)

    hsv1 = np.array([20, 100, 140], dtype=np.uint8)
    hsv2 = np.array([ 35, 255, 255], dtype=np.uint8)
    res = cv2.inRange(img_hsv, hsv1, hsv2)

    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(6,6))
    mask = cv2.morphologyEx( res, cv2.MORPH_OPEN, kernel )
    mask = cv2.morphologyEx( mask, cv2.MORPH_CLOSE, kernel )  

    return mask


def encontrar_contornos(mask):
    """
	Função que encontra contornos.
	""" 
    contornos, arvore = cv2.findContours(mask.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    return contornos
   

def crosshair(img, point, size, color):
    """ Desenha um crosshair centrado no point.
        point deve ser uma tupla (x,y)
        color é uma tupla R,G,B uint8
    """
    x,y = point
    cv2.line(img,(x - size,y),(x + size,y),color,2)
    cv2.line(img,(x,y - size),(x, y + size),color,2)

def desenhar_linha_entre_pontos(img, X, Y, color):
    """Não mude ou renomeie esta função
        deve receber uma lista de coordenadas XY, e retornar uma imagem com uma linha entre os centros EM SEQUENCIA do mais proximo.
    """
    for i in range(len(X) - 1):
        cv2.line(img,(X[i],Y[i]),(X[i+1],Y[i+1]),(0, 0, 255), 4)
    
    return img


def calcular_angulo_com_vertical(img, lm):
    """Não mude ou renomeie esta função
        deve receber uma lista de coordenadas XY, e estimar a melhor reta, utilizando o metodo preferir, que passa pelos centros. Retorne a imagem com a reta.
        
        Dica: cv2.line(img,ponto1,ponto2,color,2) desenha uma linha que passe entre os pontos, mesmo que ponto1 e ponto2 não pertençam a imagem.
    """
    m = lm[0]
    
    omega = math.atan(m)
    gama = abs(math.degrees(omega))
    angulo = 90 - gama
    
    return angulo

def center_of_mass(mask):
    """ Retorna uma tupla (cx, cy) que desenha o centro do contorno"""
    M = cv2.moments(mask)
    if M["m00"] == 0:
        return None
    cX = int(M["m10"] / M["m00"])
    cY = int(M["m01"] / M["m00"])
    return [int(cX), int(cY)]
