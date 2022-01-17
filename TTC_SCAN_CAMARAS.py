#!/usr/bin/env python
# -*- coding: utf-8 -*-
#Librerias de visión
import pyrealsense2 as rs
import numpy as np
import pandas as pd
import cv2
import imutils
from PIL import Image
#LIbrerias especiales
import json
import logging
#Librerias generales
import serial
import time
from datetime import datetime
import psutil
import math
from enum import IntEnum
from os import makedirs, remove, chdir, path, getcwd, listdir
from os.path import exists, join
import glob
import subprocess
import shutil
import MySQLdb as mdb
import matplotlib.pyplot as plt
from matplotlib import cm, rcParams
from mpl_toolkits.mplot3d import Axes3D 
from PyQt5 import QtGui
import smtplib, ssl
from email.mime.text import MIMEText
from email.mime.multipart import MIMEMultipart

#Evitar error generado por las fuentes de Matplotlib
logging.getLogger('matplotlib.font_manager').disabled = True

class TTC_PORTAL():
#Iniciando Variables
    # Semáforo
    encender_verde = 'S31'
    apagar_verde = 'S30'
    encender_rojo = 'S21'
    apagar_rojo = 'S20'
    encender_lidar = 'S41'
    reset_PC = 'RST'

    EjeX = []
    EjeY = []
    EjeZ = []

    EjeX2 = []
    EjeY2 = []
    EjeZ2 = []

    EjeX3 = []
    EjeY3 = []
    EjeZ3 = []

    EjeX4 = []
    EjeY4 = []
    EjeZ4 = []

    circles = []

    t=1
    while t<=63:
        circles.append(20 * t) 
        t+=1
                
    text = []

    k=1
    while k<=63:
        text.append(20 * k) 
        k+=1
    
    tiempo_datos_por_linea = []

    path = ''
    path_respaldo = ''

    #Iniciar Factores lidar en cero
    factor_1 = 0

    #Captura Datos para insertar en DB
    IDMEDICION = 0
    PES_ID = 0
    MP_PATENTE = 0
    MP_PATENTE_CARRO = 0
    MRCAMION_AP = 0
    MRCARRO_AP = 0
    TOTALMR_AP = 0
    MRCAMION_TR_AP = 0
    MRCARRO_TR_AP = 0
    TOTALMR_TR_AP = 0
    INICIOCABINA_AP = 0
    INICIOCAMION_AP = 0
    FINCAMION_AP = 0
    INICIOCARRO_AP = 0
    FINCARRO_AP = 0
    ANCHOCAMION = 0
    LARGOCAMION = 0
    ANCHOCARRO = 0
    LARGOCARRO = 0

    #Variables nuevas para ancho calculado
    ANCHOCAMIONCALCULADO = 0
    ANCHOCARROCALCULADO = 0
    DISTANCIACAMIONBOTTOM_A = 0
    DISTANCIACAMIONBOTTOM_B = 0
    DISTANCIACARROBOTTOM_A = 0
    DISTANCIACARROBOTTOM_B = 0
    DISTANCIACAMIONRIGHT_A = 0
    DISTANCIACAMIONRIGHT_B = 0
    DISTANCIACARRORIGHT_A = 0
    DISTANCIACARRORIGHT_B = 0

    #Variables para ancho invertido
    BASE_1_CAMION_INV = 0
    BASE_2_CAMION_INV = 0
    BASE_1_CARRO_INV = 0
    BASE_2_CARRO_INV = 0

    #Datos Medición estereo con largo suministrado
    ALTURA_CAMION_E = 0
    LARGO_CAMION_E = 0
    BASE_1_CAMION_E = 0
    BASE_2_CAMION_E = 0
    ANCHO_CAMION_E = 0
    MR_CAMION_E = 0
    ALTURA_CARRO_E = 0
    LARGO_CARRO_E = 0
    BASE_1_CARRO_E = 0
    BASE_2_CARRO_E = 0
    ANCHO_CARRO_E = 0
    MR_CARRO_E = 0
    MR_TOTAL_E = 0

    #Datos Medición con largo calculado
    VELOCIDAD_PROMEDIO_TOTAL = 0
    ALTURA_CAMION_LC = 0
    BASE_1_CAMION_LC = 0
    BASE_2_CAMION_LC = 0
    ANCHO_CAMION_LC = 0
    LARGO_CAMION_LC = 0
    MR_CAMION_LC = 0
    ALTURA_CARRO_LC = 0
    BASE_1_CARRO_LC = 0
    BASE_2_CARRO_LC = 0
    ANCHO_CARRO_LC = 0
    LARGO_CARRO_LC = 0
    MR_CARRO_LC = 0
    MR_TOTAL_LC = 0

    lista_graficar_camion_bottom = []
    lista_graficar_carro_bottom = []
    lista_graficar_camion_right = []
    lista_graficar_carro_right = []
    DISTANCIA_CAMION_BOTTOM = 0
    DISTANCIA_CARRO_BOTTOM = 0
    DISTANCIA_CAMION_RIGHT = 0
    DISTANCIA_CARRO_RIGHT = 0
    ANCHO_CAMION_CALCULADO = 0
    ANCHO_CARRO_CALCULADO = 0

#Fin Iniciar variables
    def reiniciar_Variables(self):
        self.EjeX = []
        self.EjeY = []
        self.EjeZ = []

        self.EjeX2 = []
        self.EjeY2 = []
        self.EjeZ2 = []

        self.EjeX3 = []
        self.EjeY3 = []
        self.EjeZ3 = []

        self.EjeX4 = []
        self.EjeY4 = []
        self.EjeZ4 = []

        self.circles = []

        self.t=1
        while self.t<=63:
            self.circles.append(20 * self.t) 
            self.t+=1
                    
        self.text = []

        self.k=1
        while self.k<=63:
            self.text.append(20 * self.k) 
            self.k+=1
        
        self.tiempo_datos_por_linea = []

        self.path = ''
        self.path_respaldo = ''

        #Iniciar Factores lidar en cero
        self.factor_1 = 0

        #Captura Datos para insertar en DB
        self.IDMEDICION = 0
        self.PES_ID = 0
        self.MP_PATENTE = 0
        self.MP_PATENTE_CARRO = 0
        self.MRCAMION_AP = 0
        self.MRCARRO_AP = 0
        self.TOTALMR_AP = 0
        self.MRCAMION_TR_AP = 0
        self.MRCARRO_TR_AP = 0
        self.TOTALMR_TR_AP = 0
        self.INICIOCABINA_AP = 0
        self.INICIOCAMION_AP = 0
        self.FINCAMION_AP = 0
        self.INICIOCARRO_AP = 0
        self.FINCARRO_AP = 0
        self.ANCHOCAMION = 0
        self.LARGOCAMION = 0
        self.ANCHOCARRO = 0
        self.LARGOCARRO = 0

        #Variables nuevas para ancho calculado
        self.ANCHOCAMIONCALCULADO = 0
        self.ANCHOCARROCALCULADO = 0
        self.DISTANCIACAMIONBOTTOM_A = 0
        self.DISTANCIACAMIONBOTTOM_B = 0
        self.DISTANCIACARROBOTTOM_A = 0
        self.DISTANCIACARROBOTTOM_B = 0
        self.DISTANCIACAMIONRIGHT_A = 0
        self.DISTANCIACAMIONRIGHT_B = 0
        self.DISTANCIACARRORIGHT_A = 0
        self.DISTANCIACARRORIGHT_B = 0

        #Variables para ancho invertido
        self.BASE_1_CAMION_INV = 0
        self.BASE_2_CAMION_INV = 0
        self.BASE_1_CARRO_INV = 0
        self.BASE_2_CARRO_INV = 0

        #Datos Medición estereo con largo suministrado
        self.ALTURA_CAMION_E = 0
        self.LARGO_CAMION_E = 0
        self.BASE_1_CAMION_E = 0
        self.BASE_2_CAMION_E = 0
        self.ANCHO_CAMION_E = 0
        self.MR_CAMION_E = 0
        self.ALTURA_CARRO_E = 0
        self.LARGO_CARRO_E = 0
        self.BASE_1_CARRO_E = 0
        self.BASE_2_CARRO_E = 0
        self.ANCHO_CARRO_E = 0
        self.MR_CARRO_E = 0
        self.MR_TOTAL_E = 0

        #Datos Medición con largo calculado
        self.VELOCIDAD_PROMEDIO_TOTAL = 0
        self.ALTURA_CAMION_LC = 0
        self.BASE_1_CAMION_LC = 0
        self.BASE_2_CAMION_LC = 0
        self.ANCHO_CAMION_LC = 0
        self.LARGO_CAMION_LC = 0
        self.MR_CAMION_LC = 0
        self.ALTURA_CARRO_LC = 0
        self.BASE_1_CARRO_LC = 0
        self.BASE_2_CARRO_LC = 0
        self.ANCHO_CARRO_LC = 0
        self.LARGO_CARRO_LC = 0
        self.MR_CARRO_LC = 0
        self.MR_TOTAL_LC = 0

        self.lista_graficar_camion_bottom = []
        self.lista_graficar_carro_bottom = []
        self.lista_graficar_camion_right = []
        self.lista_graficar_carro_right = []
        self.DISTANCIA_CAMION_BOTTOM = 0
        self.DISTANCIA_CARRO_BOTTOM = 0
        self.DISTANCIA_CAMION_RIGHT = 0
        self.DISTANCIA_CARRO_RIGHT = 0
        self.ANCHO_CAMION_CALCULADO = 0
        self.ANCHO_CARRO_CALCULADO = 0
    
    def TomarFotoDVR(self,cam, nombre):
        try:
            if not exists(self.path+"/Img/Imagen-Camion"):
                makedirs(self.path+"/Img/Imagen-Camion")
            ruta_directorio = self.obtener_ruta_actual()
            ruta_configuraciones = ruta_directorio+"/SETUP/configuraciones.json"
            accesos = self.cargar_datos(ruta_configuraciones, "accesos-dvr")
            ip = self.cargar_datos(ruta_configuraciones, "ip-dvr")
            cap = cv2.VideoCapture('rtsp://'+accesos+'@'+ip+':554/cam/realmonitor?channel='+cam+'&subtype=0')

            if cap.isOpened():
                ret, frame = cap.read()
                # is_success, im_buf_arr = cv2.imencode(ruta+"/"+nombre+".jpg", frame)
                # im_buf_arr.tofile(ruta+"/"+nombre+".jpg")
                cv2.imwrite(self.path+"/Img/Imagen-Camion/"+nombre+".jpg",frame)
                cap.release()
                cv2.destroyAllWindows()
            else:
                self.escribirArchivoLog("Error al iniciar Cámara DVR")
        except Exception as e:
            self.escribirArchivoLog("Error Sacar Foto del DVR: "+str(e))

    def cargar_datos(self,archivo : str, dato : str):
            with open(archivo) as contenido:
                configuraciones = json.load(contenido)[dato]
                return configuraciones

    def escribirArchivoLog(self,mensaje):
        logging.basicConfig(level=logging.DEBUG,
                        format='%(asctime)s ===> %(levelname)s ===> TTC_SCAN_Portal.exe ===> %(message)s',
                        filename = 'E:/TTC/TTC_SCAN/SETUP/Log-Errores.log',
                        filemode = 'a',)
        logging.info(mensaje)

    def leerArchivoModo(self,path,bits):
        rT = open(path,"r")
        y = rT.read(bits)
        rT.close()
        return y

    def leerArchivo(self,path):
        rT = open(path,"r")
        y = rT.read()
        rT.close()
        return y

    def escribirArchivoModo(self,x):
        ruta_directorio = self.obtener_ruta_actual()
        ruta_configuraciones = ruta_directorio+"/SETUP/configuraciones.json"
        path = self.cargar_datos(ruta_configuraciones, "Path-Modo")
        f = open(path,"w")
        f.write(x)
        f.close()

    def crear_json(self, nombre : str, datos : dict):
        rutaNombre = self.path+'/'+nombre+'.json'
        with open(rutaNombre, 'w') as archivo:
            json.dump(datos, archivo)

    def escribirArchivo(self,mensaje,cam):
        f = open(self.path+"/Puntos"+cam+".txt", "a")
        f.write(mensaje)
        f.close()

    def post_lines_in_file(self,lineas : list, name : str,path = path):
        try:
            for linea in lineas:
                f = open(path+'/'+name+'.txt', "a")
                f.write(linea+'\n')
                f.close()
        except Exception as e:
            self.escribirArchivoLog("Error Leer Archivo Datos-Camion.txt: "+str(e))

    def segundo_del_dia(self):
        try:
            hoy = datetime.today()
            hora = hoy.strftime("%H-%M-%S")
            datos = hora.split('-')
            resultado = (int(datos[0])*3600)+(int(datos[1])*60)+int(datos[2])
            return resultado
        except Exception as e:
            self.escribirArchivoLog("Error Generar segundo del día: "+str(e))

    def generated_file_full(self,tiempos : list,top : list,corner : list,bottom : list) -> None:
        try:
            ruta_directorio = self.obtener_ruta_actual()
            ruta_configuraciones = ruta_directorio+"/SETUP/configuraciones.json"
            lineas = self.get_lines_in_file(self.cargar_datos(ruta_configuraciones, 'Path-Pesaje'))
            num_pesaje = lineas[0].split('=')
            nombre = num_pesaje[1]
            const = 1
            top_list_temp = []
            # corner_list_temp = []
            bottom_list_temp = []
            slicing_inicial = 0
            slicing_final = 29
            slicing_aumentar = 29
            while slicing_final <= len(top):
                top_list_temp.append(top[slicing_inicial:slicing_final])
                slicing_inicial = slicing_final
                slicing_final+=slicing_aumentar
            slicing_inicial = 0
            slicing_final = 29
            """ while slicing_final <= len(corner):
                corner_list_temp.append(corner[slicing_inicial:slicing_final])
                slicing_inicial = slicing_final
                slicing_final+=slicing_aumentar
            slicing_inicial = 0
            slicing_final = 29 """
            while slicing_final <= len(bottom):
                bottom_list_temp.append(bottom[slicing_inicial:slicing_final])
                slicing_inicial = slicing_final
                slicing_final+=slicing_aumentar
            slicing_inicial = 0
            slicing_final = 29

            for i, tiempo in enumerate(tiempos):
                f = open(self.path+'/'+nombre+'.txt', "a")
                mensaje_inicio = '{};{};{}'
                mensaje_top = ';{};{};{};{};{};{};{};{};{};{};{};{};{};{};{};{};{};{};{};{};{};{};{};{};{};{};{};{};{}'
                # mensaje_corner = ';{};{};{};{};{};{};{};{};{};{};{};{};{};{};{};{};{};{};{};{};{};{};{};{};{};{};{};{};{}'
                mensaje_bottom = ';{};{};{};{};{};{};{};{};{};{};{};{};{};{};{};{};{};{};{};{};{};{};{};{};{};{};{};{};{}\n'
                mensaje = mensaje_inicio+mensaje_top+mensaje_bottom
                f.write(mensaje.format(const,'0',tiempo,
                top_list_temp[i][0],top_list_temp[i][1],top_list_temp[i][2],top_list_temp[i][3],top_list_temp[i][4],
                top_list_temp[i][5],top_list_temp[i][6],top_list_temp[i][7],top_list_temp[i][8],top_list_temp[i][9],
                top_list_temp[i][10],top_list_temp[i][11],top_list_temp[i][12],top_list_temp[i][13],top_list_temp[i][14],
                top_list_temp[i][15],top_list_temp[i][16],top_list_temp[i][17],top_list_temp[i][18],top_list_temp[i][19],
                top_list_temp[i][20],top_list_temp[i][21],top_list_temp[i][22],top_list_temp[i][23],top_list_temp[i][24],
                top_list_temp[i][25],top_list_temp[i][26],top_list_temp[i][27],top_list_temp[i][28],
                bottom_list_temp[i][0],bottom_list_temp[i][1],bottom_list_temp[i][2],bottom_list_temp[i][3],bottom_list_temp[i][4],
                bottom_list_temp[i][5],bottom_list_temp[i][6],bottom_list_temp[i][7],bottom_list_temp[i][8],bottom_list_temp[i][9],
                bottom_list_temp[i][10],bottom_list_temp[i][11],bottom_list_temp[i][12],bottom_list_temp[i][13],bottom_list_temp[i][14],
                bottom_list_temp[i][15],bottom_list_temp[i][16],bottom_list_temp[i][17],bottom_list_temp[i][18],bottom_list_temp[i][19],
                bottom_list_temp[i][20],bottom_list_temp[i][21],bottom_list_temp[i][22],bottom_list_temp[i][23],bottom_list_temp[i][24],
                bottom_list_temp[i][25],bottom_list_temp[i][26],bottom_list_temp[i][27],bottom_list_temp[i][28]))
                f.close()
                const+=1

        except Exception as e:
            self.escribirArchivoLog("Error Generar archivo Numero Pesaje FULL.txt: "+str(e))

    def get_lines_in_file(self,path : str) -> list:
        try:
            x=[]
            file = open(path,'r')
            lineas = file.readlines()
            file.close()

            for linea in lineas:
                x.append(linea.rstrip('\n'))
            return x
        except Exception as e:
            self.escribirArchivoLog("Error Leer Archivo NPesaje: "+str(e))

    def get_lines_in_file_dict(self, path : str) -> dict:
        try:
            y={}
            file = open(path,'r')
            lineas = file.readlines()
            file.close()

            for linea in lineas:
                temp_list = linea.rstrip('\n').split('=')
                y[temp_list[0]] = temp_list[1]
                temp_list=[]
            return y
        except Exception as e:
            self.escribirArchivoLog("Error Leer Archivo NPesaje: "+str(e))

    def get_distance_from_lidar(self,ser : object) -> str:
        try:
            x = ser.readline()
            modo = x.decode('utf-8')
            modo=modo.split(',')
            distancia = modo[-1]
            distancia = distancia[:-4]
            return distancia
        except:
            #escribirArchivoLog("Error Leer Puerto Serial: "+str(e))
            pass

    def crearCarpetaData(self):
        ruta_directorio = self.obtener_ruta_actual()
        ruta_configuraciones = ruta_directorio+"/SETUP/configuraciones.json"
        lineas = self.get_lines_in_file(self.cargar_datos(ruta_configuraciones, 'Path-Pesaje'))
        num_pesaje = lineas[0].split('=')
        num_pesaje = num_pesaje[1]
        ruta_camiones = self.cargar_datos(ruta_configuraciones, 'Ruta-Camiones')
        ruta_camiones_respaldo = self.cargar_datos(ruta_configuraciones, 'Ruta-Camiones-Respaldo')
        respaldo_fotos = self.cargar_datos(ruta_configuraciones,"Respaldo-Fotos")
        self.path = ruta_camiones+"/"+str(num_pesaje)
        self.path_respaldo = ruta_camiones_respaldo+"/"+str(num_pesaje)

        # Creando carpeta para ruta principal
        if not exists(self.path):
            makedirs(self.path)
            makedirs(self.path+"/Img")
        else:
            #print('Ya existe la carpeta se va a borrar',self.path)
            shutil.rmtree(self.path, ignore_errors=True)
            makedirs(self.path)
            makedirs(self.path+"/Img")

        # Creando carpeta para ruta respaldo
        if respaldo_fotos == True:
            if not exists(self.path_respaldo):
                makedirs(self.path_respaldo)
                makedirs(self.path_respaldo+"/Img")
            else:
                #print('Ya existe la carpeta se va a borrar',self.path)
                shutil.rmtree(self.path_respaldo, ignore_errors=True)
                makedirs(self.path_respaldo)
                makedirs(self.path_respaldo+"/Img")

    def check_cameras_on(self):
        
        self.escribirArchivoLog("CHECK CAMERAS START")
        Preset = {
            'Custom' : 0,
            'Default' : 1,
            'Hand' : 2,
            'HighAccuracy' : 3,
            'HighDensity' : 4,
            'MediumDensity' : 5
        }

        
        try:            
            ruta_directorio = self.obtener_ruta_actual()
            ruta_configuraciones = ruta_directorio+"/SETUP/configuraciones.json"
            RS_TOP_ACTIVE = self.cargar_datos(ruta_configuraciones,"RS-TOP-ACTIVE")
            RS_CORNER_ACTIVE = self.cargar_datos(ruta_configuraciones,"RS-CORNER-ACTIVE")
            RS_BOTTOM_ACTIVE = self.cargar_datos(ruta_configuraciones,"RS-BOTTOM-ACTIVE")
            RS_RIGHT_ACTIVE = self.cargar_datos(ruta_configuraciones,"RS-RIGHT-ACTIVE")
            if RS_TOP_ACTIVE == True:
                pipeline = rs.pipeline()
                config = rs.config()
                config.enable_device(self.cargar_datos(ruta_configuraciones,"RS-TOP-DEVICE"))
                profile = pipeline.start(config)
            else:
                pass

            if RS_CORNER_ACTIVE == True:
                pipeline2 = rs.pipeline()
                config2 = rs.config()
                config2.enable_device(self.cargar_datos(ruta_configuraciones,"RS-CORNER-DEVICE"))
                profile2 = pipeline2.start(config2)
            else:
                pass

            if RS_BOTTOM_ACTIVE == True:
                pipeline3 = rs.pipeline()
                config3 = rs.config()
                config3.enable_device(self.cargar_datos(ruta_configuraciones,"RS-BOTTOM-DEVICE"))
                profile3 = pipeline3.start(config3)
            else:
                pass

            if RS_RIGHT_ACTIVE == True:
                pipeline4 = rs.pipeline()
                config4 = rs.config()
                config4.enable_device(self.cargar_datos(ruta_configuraciones,"RS-RIGHT-DEVICE"))
                profile4 = pipeline4.start(config4)
            else:
                pass

            if RS_TOP_ACTIVE == True:
                pipeline.stop()                
            else:
                pass

            if RS_CORNER_ACTIVE == True:
                pipeline2.stop()            
            else:
                pass

            if RS_BOTTOM_ACTIVE == True:
                pipeline3.stop()      
            else:
                pass

            if RS_RIGHT_ACTIVE == True:
                pipeline4.stop()      
            else:
                pass
        
            self.escribirArchivoLog("CHECK CAMERAS OK")
            
            
            return True

        except Exception as e:
            self.post_archivo_txt("CAMERAS-NOT-WORK")
            self.escribirArchivoLog("Error Cameras OFF al iniciar: "+str(e))
            self.enviar_mensaje_correo("Error de las cámaras al iniciar sistema, \
            Debe acceder a solucionar el problema manualmente")
            self.escribirArchivoLog("CHECK CAMERAS ERROR")

            return False

            # self.write_txt("R:/TTC_SCAN/Modo.txt","t")
            # time.sleep(3)
            # subprocess.call("shutdown -s")

    def realsense_hardware_resetOLD(self):
        
        try:
            self.escribirArchivoLog("HARDWARE RESET START")
            ruta_directorio = self.obtener_ruta_actual()
            ruta_configuraciones = ruta_directorio+"/SETUP/configuraciones.json"
            RS_TOP_ACTIVE = self.cargar_datos(ruta_configuraciones,"RS-TOP-ACTIVE")
            RS_BOTTOM_ACTIVE = self.cargar_datos(ruta_configuraciones,"RS-BOTTOM-ACTIVE")
            RS_RIGHT_ACTIVE = self.cargar_datos(ruta_configuraciones,"RS-RIGHT-ACTIVE")
            if RS_TOP_ACTIVE == True:
                pipeline = rs.pipeline()
                config = rs.config()
                config.enable_device(self.cargar_datos(ruta_configuraciones,"RS-TOP-DEVICE"))
                profile = pipeline.start(config)
                profile.get_device().hardware_reset()
                time.sleep(5)
                print("HARDWARE RESET CAM 1 OK")
                self.escribirArchivoLog("HARDWARE RESET CAM 1 OK")
            else:
                pass

            if RS_BOTTOM_ACTIVE == True:
                pipeline3 = rs.pipeline()
                config3 = rs.config()
                config3.enable_device(self.cargar_datos(ruta_configuraciones,"RS-BOTTOM-DEVICE"))
                profile3 = pipeline3.start(config3)
                profile3.get_device().hardware_reset()
                time.sleep(5)
                print("HARDWARE RESET CAM 3 OK")
                self.escribirArchivoLog("HARDWARE RESET CAM 3 OK")
            else:
                pass

            if RS_RIGHT_ACTIVE == True:
                pipeline4 = rs.pipeline()
                config4 = rs.config()
                config4.enable_device(self.cargar_datos(ruta_configuraciones,"RS-RIGHT-DEVICE"))
                profile4 = pipeline4.start(config4)
                profile4.get_device().hardware_reset()
                time.sleep(5)
                print("HARDWARE RESET CAM 4 OK")
                self.escribirArchivoLog("HARDWARE RESET CAM 4 OK")
            else:
                pass
            
            # if RS_TOP_ACTIVE == True:
            #     print("FINALIZANDO PROCESO -> HARDWARE RESET CAM 1 OK")
            #     self.escribirArchivoLog("FINALIZANDO PROCESO -> HARDWARE RESET CAM 1 OK")
            #     pipeline.stop()                
            # else:
            #     pass

            # if RS_BOTTOM_ACTIVE == True:
            #     print("FINALIZANDO PROCESO -> HARDWARE RESET CAM 3 OK")
            #     self.escribirArchivoLog("FINALIZANDO PROCESO -> HARDWARE RESET CAM 3 OK")
            #     pipeline3.stop()                
            # else:
            #     pass

            # if RS_RIGHT_ACTIVE == True:
            #     print("FINALIZANDO PROCESO -> HARDWARE RESET CAM 4 OK")
            #     self.escribirArchivoLog("FINALIZANDO PROCESO -> HARDWARE RESET CAM 4 OK")
            #     pipeline4.stop()             
            # else:
            #     pass

            
            self.escribirArchivoLog("HARDWARE RESET OK")
            print("OK Hardware Reset")
            return True
        
        except Exception as e:
            self.escribirArchivoLog("Error hardware reset: "+str(e))
            self.enviar_mensaje_correo("Hardware Reset falló se cortará la energía.")
            print("Error Hardware Reset")
            self.escribirArchivoLog("HARDWARE RESET ERROR")
            return False

    def realsense_hardware_reset(self):
        self.escribirArchivoLog("HARDWARE RESET START")
        try:    
            ctx = rs.context()
            dev_list = ctx.query_devices()
            for dev in dev_list:
                try:
                    dev.hardware_reset()
                    # self.escribirArchivoLog("HARDWARE RESET OK Device:{}".format(dev))
                    print("HARDWARE RESET OK Device:{}".format(dev))
                except:
                    self.escribirArchivoLog("HARDWARE RESET ERROR Device:{}".format(dev))
                    #Reset por modulo
                    self.write_serial(self.reset_PC)
                    #Reset por Raspberry 
                    #self.write_txt("R:/TTC_SCAN/Modo.txt","t")
            time.sleep(1)
            
            self.escribirArchivoLog("HARDWARE RESET OK")
            print("OK Hardware Reset")
            return True
        
        except Exception as e:
            self.escribirArchivoLog("Error hardware reset: "+str(e))
            self.enviar_mensaje_correo("Hardware Reset falló se cortará la energía.")
            print("Error Hardware Reset")
            self.escribirArchivoLog("HARDWARE RESET ERROR")
            #Reset por modulo
            self.write_serial(self.reset_PC)
            #Reset por Raspberry 
            #self.write_txt("R:/TTC_SCAN/Modo.txt","t")
            return False

    def iniciarRecoleccionDeDistancias(self):
        Preset = {
            'Custom' : 0,
            'Default' : 1,
            'Hand' : 2,
            'HighAccuracy' : 3,
            'HighDensity' : 4,
            'MediumDensity' : 5
        }

        def getDistance(x,y):
            D = depth.get_distance(x,y)
            D*=100
            return D

        def getDistance2(x,y):
            D = depth2.get_distance(x,y)
            D*=100
            return D

        def getDistance3(x,y):
            D = depth3.get_distance(x,y)
            D*=100
            return D

        def getDistance4(x,y):
            D = depth4.get_distance(x,y)
            D*=100
            return D
        
        try:


            
            ruta_directorio = self.obtener_ruta_actual()
            ruta_configuraciones = ruta_directorio+"/SETUP/configuraciones.json"
            RS_TOP_ACTIVE = self.cargar_datos(ruta_configuraciones,"RS-TOP-ACTIVE")
            RS_CORNER_ACTIVE = self.cargar_datos(ruta_configuraciones,"RS-CORNER-ACTIVE")
            RS_BOTTOM_ACTIVE = self.cargar_datos(ruta_configuraciones,"RS-BOTTOM-ACTIVE")
            RS_RIGHT_ACTIVE = self.cargar_datos(ruta_configuraciones,"RS-RIGHT-ACTIVE")

            if RS_TOP_ACTIVE == True:
                pipeline = rs.pipeline()
                config = rs.config()
                config.enable_device(self.cargar_datos(ruta_configuraciones,"RS-TOP-DEVICE"))
                config.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 30)
                config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)
                profile = pipeline.start(config)
                depth_sensor = profile.get_device().first_depth_sensor()
                depth_sensor.set_option(rs.option.visual_preset, Preset['HighDensity'])
            else:
                pass

            if RS_CORNER_ACTIVE == True:
                pipeline2 = rs.pipeline()
                config2 = rs.config()
                config2.enable_device(self.cargar_datos(ruta_configuraciones,"RS-CORNER-DEVICE"))
                config2.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 30)
                config2.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)
                profile2 = pipeline2.start(config2)
                depth_sensor2 = profile2.get_device().first_depth_sensor()
                depth_sensor2.set_option(rs.option.visual_preset, Preset['HighDensity'])
            else:
                pass

            if RS_BOTTOM_ACTIVE == True:
                pipeline3 = rs.pipeline()
                config3 = rs.config()
                config3.enable_device(self.cargar_datos(ruta_configuraciones,"RS-BOTTOM-DEVICE"))
                config3.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 30)
                config3.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)
                profile3 = pipeline3.start(config3)
                depth_sensor3 = profile3.get_device().first_depth_sensor()
                depth_sensor3.set_option(rs.option.visual_preset, Preset['HighDensity'])
            else:
                pass

            if RS_RIGHT_ACTIVE == True:
                pipeline4 = rs.pipeline()
                config4 = rs.config()
                config4.enable_device(self.cargar_datos(ruta_configuraciones,"RS-RIGHT-DEVICE"))
                config4.enable_stream(rs.stream.depth, 1280, 720, rs.format.z16, 30)
                config4.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)
                profile4 = pipeline4.start(config4)
                depth_sensor4 = profile4.get_device().first_depth_sensor()
                depth_sensor4.set_option(rs.option.visual_preset, Preset['HighDensity'])
            else:
                pass
                       
            contadorX = 1
            contadorX2 = 1
            contadorX3 = 1
            contadorX4 = 1
            
            respaldo_fotos = False

            tiempo = self.cargar_datos(ruta_configuraciones,"Tiempo")
            respaldo_fotos = self.cargar_datos(ruta_configuraciones,"Respaldo-Fotos")

            tiempo_inicio = time.time()

            # Validador para el IF que se ejecuta una vez y la primera vez
            validar = True

            validar_apagar_semaforo = True

            tempProj = 0
            tempAsic = 0
            tempProj2 = 0
            tempAsic2 = 0
            tempProj3 = 0
            tempAsic3 = 0
            tempProj4 = 0
            tempAsic4 = 0

            estadoProceso = True

            #HARDWARE RESET
            evaluacion_cam_1 = True
            evaluacion_cam_2 = True
            evaluacion_cam_3 = True
            evaluacion_cam_4 = True

            while True:
                
                if RS_TOP_ACTIVE == True:
                    frames = pipeline.wait_for_frames()
                    depth = frames.get_depth_frame()
                    color_frame = frames.get_color_frame()
                    if not depth or not color_frame: continue
                    color_image = np.asanyarray(color_frame.get_data())
                    # Extracto para guardar imagenes respaldo al natural
                    if respaldo_fotos == True:
                        if not exists(self.path_respaldo+"/Img/Camara-Top"):
                            makedirs(self.path_respaldo+"/Img/Camara-Top")                  
                        cv2.imwrite(self.path_respaldo+"/Img/Camara-Top/"+str(contadorX)+".jpg",color_image)

                    # Obtener temperatura de camara
                    # Get device temperature
                    tempProj = depth_sensor.get_option(rs.option.projector_temperature)
                    # print("tempProj: " + str(tempProj))
                    tempAsic = depth_sensor.get_option(rs.option.asic_temperature)
                    # print("tempAsic: " + str(tempAsic ))

                    for x in self.circles:
                        y=360
                        # print(x,y)
                        cv2.circle(color_image, (x,y), 2, (255, 255, 255), 5)
                        cv2.circle(color_image, (x,y), 2, (0, 0, 255), 2)
                    color = (0,0,255)
                    fsize = 0.3
                    bold = 0
                    font = cv2.FONT_HERSHEY_SIMPLEX
                    cons=1
                    for i, x in enumerate(self.text):
                        y = 360
                        # print(x,y)
                        D = getDistance(x,y)
                        num = i % 2
                        if num == 0:
                            var = 40
                        else:
                            var = 80
                        cv2.putText(color_image, str(cons)+":"+str(int(D)), (x,y-var), font,fsize, (255, 255, 255), 3,cv2.LINE_AA)
                        cv2.putText(color_image, str(cons)+":"+str(int(D)), (x,y-var),font,fsize,color,1,cv2.LINE_AA)
                        cons+=1  
                    if not exists(self.path+"/Img/Camara-Top"):
                        makedirs(self.path+"/Img/Camara-Top")                  
                    cv2.imwrite(self.path+"/Img/Camara-Top/"+str(contadorX)+".jpg",color_image)

                    for Xeje in self.circles:
                        Yeje=360
                        Zeje = getDistance(Xeje,Yeje)
                        self.EjeX.append(contadorX)
                        self.EjeY.append(Yeje)
                        self.EjeZ.append(Zeje)
                    contadorX+=1
                else:
                    pass

                if RS_CORNER_ACTIVE == True:
                    frames2 = pipeline2.wait_for_frames()
                    depth2 = frames2.get_depth_frame()
                    color_frame2 = frames2.get_color_frame()
                    if not depth2 or not color_frame2: continue
                    color_image2 = np.asanyarray(color_frame2.get_data())
                    # Extracto para guardar imagenes respaldo al natural
                    if respaldo_fotos == True:
                        if not exists(self.path_respaldo+"/Img/Camara-Corner"):
                            makedirs(self.path_respaldo+"/Img/Camara-Corner")                  
                        cv2.imwrite(self.path_respaldo+"/Img/Camara-Corner/"+str(contadorX2)+".jpg",color_image2)

                    # Obtener temperatura de camara
                    # Get device temperature
                    tempProj2 = depth_sensor2.get_option(rs.option.projector_temperature)
                    # print("tempProj2: " + str(tempProj2))
                    tempAsic2 = depth_sensor2.get_option(rs.option.asic_temperature)
                    # print("tempAsic2: " + str(tempAsic2 ))

                    for x in self.circles:
                        y=360
                        # print(x,y)
                        cv2.circle(color_image2, (x,y), 2, (255, 255, 255), 5)
                        cv2.circle(color_image2, (x,y), 2, (0, 0, 255), 2)
                    color = (0,0,255)
                    fsize = 0.3
                    bold = 0
                    font = cv2.FONT_HERSHEY_SIMPLEX
                    cons=1
                    for i, x in enumerate(self.text):
                        y = 360
                        # print(x,y)
                        D = getDistance2(x,y)
                        num = i % 2
                        if num == 0:
                            var = 40
                        else:
                            var = 80
                        cv2.putText(color_image2, str(cons)+":"+str(int(D)), (x,y-var), font,fsize, (255, 255, 255), 3,cv2.LINE_AA)
                        cv2.putText(color_image2, str(cons)+":"+str(int(D)), (x,y-var),font,fsize,color,1,cv2.LINE_AA)
                        cons+=1                 
                    if not exists(self.path+"/Img/Camara-Corner"):
                        makedirs(self.path+"/Img/Camara-Corner")    
                    cv2.imwrite(self.path+"/Img/Camara-Corner/"+str(contadorX2)+".jpg",color_image2)
                    for Xeje in self.circles:
                        Yeje=360
                        Zeje = getDistance2(Xeje,Yeje)
                        self.EjeX2.append(contadorX2)
                        self.EjeY2.append(Yeje)
                        self.EjeZ2.append(Zeje)
                    contadorX2+=1
                else:
                    pass

                if RS_BOTTOM_ACTIVE == True:
                    rotar_imagen = self.cargar_datos(ruta_configuraciones,"RS-BOTTOM-ROTAR")
                    frames3 = pipeline3.wait_for_frames()
                    depth3 = frames3.get_depth_frame()
                    color_frame3 = frames3.get_color_frame()
                    if not depth3 or not color_frame3: continue
                    color_image3 = np.asanyarray(color_frame3.get_data())
                    # Extracto para guardar imagenes respaldo al natural
                    if respaldo_fotos == True:
                        if not exists(self.path_respaldo+"/Img/Camara-Bottom"):
                            makedirs(self.path_respaldo+"/Img/Camara-Bottom")                  
                        cv2.imwrite(self.path_respaldo+"/Img/Camara-Bottom/"+str(contadorX3)+".jpg",color_image3)

                    # Obtener temperatura de camara
                    # Get device temperature
                    tempProj3 = depth_sensor3.get_option(rs.option.projector_temperature)
                    # print("tempProj3: " + str(tempProj3))
                    tempAsic3 = depth_sensor3.get_option(rs.option.asic_temperature)
                    # print("tempAsic3: " + str(tempAsic3 ))

                    for x in self.circles: 
                        y=360
                        # print(x,y)
                        cv2.circle(color_image3, (x,y), 2, (255, 255, 255), 5)
                        cv2.circle(color_image3, (x,y), 2, (0, 0, 255), 2)
                    color = (0,0,255)
                    fsize = 0.3
                    bold = 0
                    font = cv2.FONT_HERSHEY_SIMPLEX
                    cons=1
                    for i, x in enumerate(self.text):
                        y = 360
                        # print(x,y)
                        D = getDistance3(x,y)
                        num = i % 2
                        if num == 0:
                            var = 40
                        else:
                            var = 80
                        cv2.putText(color_image3, str(cons)+":"+str(int(D)), (x,y-var), font,fsize, (255, 255, 255), 3,cv2.LINE_AA)
                        cv2.putText(color_image3, str(cons)+":"+str(int(D)), (x,y-var),font,fsize,color,1,cv2.LINE_AA)
                        cons+=1       
                    # Creando carpera en ruta principal     
                    if not exists(self.path+"/Img/Camara-Bottom"):
                        makedirs(self.path+"/Img/Camara-Bottom") 
                    if rotar_imagen == True:
                        #  M = cv2.getRotationMatrix2D((1280//2,720//2),270,1)
                         color_image3 = imutils.rotate_bound(color_image3, 90)
                    else:
                        pass
                    cv2.imwrite(self.path+"/Img/Camara-Bottom/"+str(contadorX3)+".jpg",color_image3)
                    for Xeje in self.circles:
                        Yeje=360
                        Zeje = getDistance3(Xeje,Yeje)
                        self.EjeX3.append(contadorX3)
                        self.EjeY3.append(Yeje)
                        self.EjeZ3.append(Zeje)
                    contadorX3+=1
                else:
                    pass

                if RS_RIGHT_ACTIVE == True:
                    rotar_imagen = self.cargar_datos(ruta_configuraciones,"RS-RIGHT-ROTAR")
                    frames4 = pipeline4.wait_for_frames()
                    depth4 = frames4.get_depth_frame()
                    color_frame4 = frames4.get_color_frame()
                    if not depth4 or not color_frame4: continue
                    color_image4 = np.asanyarray(color_frame4.get_data())
                    # Extracto para guardar imagenes respaldo al natural
                    if respaldo_fotos == True:
                        if not exists(self.path_respaldo+"/Img/Camara-Right"):
                            makedirs(self.path_respaldo+"/Img/Camara-Right")                  
                        cv2.imwrite(self.path_respaldo+"/Img/Camara-Right/"+str(contadorX4)+".jpg",color_image4)

                    # Obtener temperatura de camara
                    # Get device temperature
                    tempProj4 = depth_sensor4.get_option(rs.option.projector_temperature)
                    # print("tempProj4: " + str(tempProj4))
                    tempAsic4 = depth_sensor4.get_option(rs.option.asic_temperature)
                    # print("tempAsic4: " + str(tempAsic4 ))

                    for x in self.circles: 
                        y=360
                        # print(x,y)
                        cv2.circle(color_image4, (x,y), 2, (255, 255, 255), 5)
                        cv2.circle(color_image4, (x,y), 2, (0, 0, 255), 2)
                    color = (0,0,255)
                    fsize = 0.3
                    bold = 0
                    font = cv2.FONT_HERSHEY_SIMPLEX
                    cons=1
                    for i, x in enumerate(self.text):
                        y = 360
                        # print(x,y)
                        D = getDistance4(x,y)
                        num = i % 2
                        if num == 0:
                            var = 40
                        else:
                            var = 80
                        cv2.putText(color_image4, str(cons)+":"+str(int(D)), (x,y-var), font,fsize, (255, 255, 255), 3,cv2.LINE_AA)
                        cv2.putText(color_image4, str(cons)+":"+str(int(D)), (x,y-var),font,fsize,color,1,cv2.LINE_AA)
                        cons+=1       
                    # Creando carpera en ruta principal     
                    if not exists(self.path+"/Img/Camara-Right"):
                        makedirs(self.path+"/Img/Camara-Right") 
                    if rotar_imagen == True:
                        #  M = cv2.getRotationMatrix2D((1280//2,720//2),270,1)
                         color_image4 = imutils.rotate_bound(color_image4, 90)
                    else:
                        pass
                    cv2.imwrite(self.path+"/Img/Camara-Right/"+str(contadorX4)+".jpg",color_image4)
                    for Xeje in self.circles:
                        Yeje=360
                        Zeje = getDistance4(Xeje,Yeje)
                        self.EjeX4.append(contadorX4)
                        self.EjeY4.append(Yeje)
                        self.EjeZ4.append(Zeje)
                    contadorX4+=1
                else:
                    pass
                
                # Este IF solo se ejecuta una vez y en el primer ciclo del
                # While
                if validar == True:
                    ##########################################
                    ##########################################
                    #*****************************************
                    #*****************************************
                    # Validar que este sin error la medición camara 1
                    num_error = self.EjeZ.count(0.0)
                    num_datos = len(self.EjeZ)
                    num_ok = num_datos - num_error

                    if num_error > num_ok:
                        evaluacion_cam_1 = False

                    # Validar que este sin error la medición camara 2
                    # num_error2 = self.EjeZ2.count(0.0)
                    # num_datos2 = len(self.EjeZ2)
                    # num_ok2 = num_datos2 - num_error2

                    # if num_error2 > num_ok2:
                    #     evaluacion_cam_2 = False
                    
                    # Validar que este sin error la medición camara 3
                    num_error3 = self.EjeZ3.count(0.0)
                    num_datos3 = len(self.EjeZ3)
                    num_ok3 = num_datos3 - num_error3

                    if num_error3 > num_ok3:
                        evaluacion_cam_3 = False
                    
                    # Validar que este sin error la medición camara 4
                    num_error4 = self.EjeZ4.count(0.0)
                    num_datos4 = len(self.EjeZ4)
                    num_ok4 = num_datos4 - num_error4

                    if num_error4 > num_ok4:
                        evaluacion_cam_4 = False


                    if (evaluacion_cam_1 == False) or (evaluacion_cam_2 == False) or (evaluacion_cam_3 == False) or (evaluacion_cam_4 == False):
                        # Ejecutar proceso de error de cámara TOP
                        # El Proceso de Daniela se detiene hasta que se haga el Hardware reset
                        # Se envía REANUDA y en 2 segundos ESPERANDO
                        self.post_archivo_txt("CAMERAS-NOT-WORK-TEMP")
                        self.escribirArchivoLog("Error camarás con error 0.0 Cam1: {} - Cam3: {} - Cam 4: {}".format(evaluacion_cam_1,evaluacion_cam_3,evaluacion_cam_4))
                        # self.enviar_mensaje_correo("Reinicio del SISTEMA TTC_SCAN ARAUCO CON ERROR CÁMARAS 0.0")
                        #Hardware Reset 
                        isOkHR = self.realsense_hardware_reset()
                        time.sleep(2)
                        if isOkHR == False:
                            self.post_archivo_txt("CAMERAS-NOT-WORK")
                            self.escribirArchivoLog("HARDWARE RESET FALSE - RESET BY MODULE")
                            #Reset por modulo
                            self.write_serial(self.reset_PC)
                            #Reset por Raspberry 
                            #self.write_txt("R:/TTC_SCAN/Modo.txt","t")
                        elif isOkHR == True:
                            self.post_archivo_txt("REANUDANDO")
                        time.sleep(2)
                        self.escribirArchivoModo("f")
                        #Variable controla la respuesta de ESTADO-SCAN al finalizar esta función
                        estadoProceso = False
                        break
                    else:
                        pass

                    #*****************************************
                    #*****************************************
                    ##########################################
                    ##########################################


                    ##########################################
                    ##########################################
                    #*****************************************
                    #*****************************************
                    #Crear archivo en unidad S:/ Iniciado scan
                    self.post_archivo_txt("ESCANEANDO")
                    self.write_serial(self.apagar_rojo)
                    time.sleep(0.5)
                    self.write_serial(self.encender_verde)
                    #Para Carlos Duplicar el NPesaje
                    npesaje = 'C:/TTCScan/NPesaje.txt'
                    if exists(npesaje):
                        remove(npesaje)
                    self.post_lines_in_file(self.get_lines_in_file(self.cargar_datos(ruta_configuraciones,'Path-Pesaje')),'NPesaje','C:/TTCScan')
                    #FIN -- Para Carlos Duplicar el NPesaje
                    validar = False
                    #*****************************************
                    #*****************************************
                    ##########################################
                    ##########################################

                self.tiempo_datos_por_linea.append(self.segundo_del_dia())

                tiempo_trancurrido = time.time() - tiempo_inicio
                # IF para apagar semaforo a los 30 SEG
                if int(tiempo_trancurrido) > 30 and validar_apagar_semaforo == True:
                    validar_apagar_semaforo = False
                    self.post_archivo_txt("APAGAR-VERDE")
                    self.write_serial(self.apagar_verde)

                # IF para continuar si el tiempo es menor a la duración del escaneo
                if int(tiempo_trancurrido) <= int(tiempo):
                    pass
                    
                # IF para terminar el ciclo y guardar los datos del escaneo
                # al llegar al tiempo limite de duración del escaneo.
                if int(tiempo_trancurrido) > int(tiempo):
                    self.escribirArchivoModo("g")
                    # Se crea un archivo llamado Info-Camaras-IDCAMION.txt
                    mensaje =  ("<--INFO-TOP-->\n"+
                                    "TEMPERATURA_SENSOR="+str(tempProj)+"\n"+
                                    "TEMPERATURA_TARJETA="+str(tempAsic)+"\n"+
                                    "<--INFO-CORNER-->\n"+
                                    "TEMPERATURA_SENSOR="+str(tempProj2)+"\n"+
                                    "TEMPERATURA_TARJETA="+str(tempAsic2)+"\n"+
                                    "<--INFO-BOTTOM-->\n"+
                                    "TEMPERATURA_SENSOR="+str(tempProj3)+"\n"+
                                    "TEMPERATURA_TARJETA="+str(tempAsic3)+"\n"+
                                    "<--INFO-RIGHT-->\n"+
                                    "TEMPERATURA_SENSOR="+str(tempProj4)+"\n"+
                                    "TEMPERATURA_TARJETA="+str(tempAsic4)+"")
                    self.post_info_camaras_txt(self.path, 'INFO-CAMARAS', mensaje)
                    break
                
            if RS_TOP_ACTIVE == True:
                pipeline.stop()                
            else:
                pass

            if RS_CORNER_ACTIVE == True:
                pipeline2.stop()                
            else:
                pass

            if RS_BOTTOM_ACTIVE == True:
                pipeline3.stop()                
            else:
                pass

            if RS_RIGHT_ACTIVE == True:
                pipeline4.stop()             
            else:
                pass

            if estadoProceso == True:
                self.post_archivo_txt("ESCANEADO")
            elif estadoProceso == False:
                self.post_archivo_txt("ESPERANDO")
            self.write_serial(self.apagar_verde)
            time.sleep(2)

            return self.EjeX, self.EjeY, self.EjeZ, \
                    self.EjeX2, self.EjeY2, self.EjeZ2, \
                    self.EjeX3, self.EjeY3, self.EjeZ3, \
                    self.EjeX4, self.EjeY4, self.EjeZ4, \
                    self.tiempo_datos_por_linea
        
        except Exception as e:
            self.post_archivo_txt("CAMERAS-NOT-WORK")
            self.escribirArchivoLog("Error al Escanear: "+str(e))
            # self.enviar_mensaje_correo("Reinicio del SISTEMA TTC_SCAN ARAUCO CON CÁMARAS OFF")
            #Hardware Reset 
            isOkHR = self.realsense_hardware_reset()
            time.sleep(2)
            if isOkHR == False:
                self.post_archivo_txt("CAMERAS-NOT-WORK")
                self.escribirArchivoLog("HARDWARE RESET FALSE - RESET BY MODULE")
                #Reset por modulo
                self.write_serial(self.reset_PC)
                #Reset por Raspberry 
                #self.write_txt("R:/TTC_SCAN/Modo.txt","t")
            elif isOkHR == True:
                self.post_archivo_txt("REANUDANDO")
            # isCheck = self.check_cameras_on()
            # if isCheck == False:
            #     self.escribirArchivoLog("CHECK CAMERAS FALSE - RESET BY MODULE")
            #     #Reset por modulo
            #     self.write_serial(self.reset_PC)
            #     #Reset por Raspberry 
            #     #self.write_txt("R:/TTC_SCAN/Modo.txt","t")
            time.sleep(2)
            self.post_archivo_txt("ESPERANDO")
            self.escribirArchivoModo("f")
                  
    def regenerete_list_z(self, lista : list) -> list:

        try:
            list_temp = []
            slicing_inicial = 0
            slicing_final = 63
            slicing_aumentar = 63
            while slicing_final <= len(lista):
                list_temp.append(lista[slicing_inicial:slicing_final])
                slicing_inicial = slicing_final
                slicing_final+=slicing_aumentar
            return list_temp

        except Exception as e:
            self.escribirArchivoLog("Error regenerete_list_z(): "+str(e))

    def promedio_alturas(self, banco : list, base = int) -> int:
        list_temp = []
        suma_list = 0
        for dato in banco:
            if dato > 10:
                list_temp.append(dato - base)
                suma_list+=dato

        prom_h = suma_list / len(list_temp)
        return round(prom_h,2)
        
    def promedio_alturas_bancos_full_estereo(self, banco : list) -> int:
        ruta_directorio = self.obtener_ruta_actual()
        ruta_configuraciones = ruta_directorio+"/SETUP/configuraciones.json"

        #Constantes
        limites_altura_matriz_banco = self.cargar_datos(ruta_configuraciones, "LIMITES-ALTURA-MATRIZ-BANCO")
        altura_portal = self.cargar_datos(ruta_configuraciones, "ALTURA-PORTAL")
        try:
            list_temp = []
            suma_list = 0
            for linea in banco:
                for dato in linea:
                    if dato > limites_altura_matriz_banco[1] and dato < limites_altura_matriz_banco[0]:
                        h = (altura_portal - dato)
                        list_temp.append(h)
                        suma_list+=h

            prom_h = suma_list / len(list_temp)
            #print(base,round(prom_h,2))
            return round(prom_h,2)
        except Exception as e:
            self.escribirArchivoLog("Error promedio_alturas_bancos_full_estereo: "+str(e))

    def promedio_dinamico(self, val : list, activador : bool):
        if activador == False:
            promedio = val[0]
        else:
            sum_valores = 0
            numero_elementos = 0
            promedio = 0
            for value in val:
                sum_valores+=value
            
            numero_elementos = len(val)

            if numero_elementos != 0:
                promedio = sum_valores / numero_elementos
            else:
                promedio = 1

        return promedio

    def calculo_metro_ruma(self, prom_h : float, ancho : float, largo : float):
        mtr_ruma = ( prom_h * ancho * largo ) / 2.44
        return round(mtr_ruma,2)
    
    def obtener_camion_carro(self,lista_original : list) -> list:
        ruta_directorio = self.obtener_ruta_actual()
        ruta_configuraciones = ruta_directorio+"/SETUP/configuraciones.json"
        lista_indice = []
        limites_altura_obtener_camion = self.cargar_datos(ruta_configuraciones,"LIMITES-ALTURA-OBTENER-CAMION")
        # print(limites_altura_obtener_camion)
        #Detectar incio cámion y fin carro
        for i, lista in enumerate(lista_original):

            
            if lista == None or lista == '':
                pass
            else:
                if int(lista) < limites_altura_obtener_camion[0] \
                    and int(lista) > limites_altura_obtener_camion[1]:
                    lista_indice.append(i)
        return lista_indice

    def detectar_fin_camion(self,lista_z : list,lista_x : list, i_carro : int, i_camion : int) -> int:
        list_temp = []
        i_cami = lista_x.index(i_camion+1)
        i_carr = lista_x.index(i_carro)

        for i, dato in enumerate(lista_x):

            if i < i_cami and i > i_carr and lista_z[i] < 400 and lista_z[i] != 0.0:
                list_temp.append(dato)
            

        indice_fin_camion = list_temp[1]

        return indice_fin_camion 

    def detectar_inicio_carro(self,lista_z : list,lista_x : list) -> int:
        list_temp = []
        lista_z.reverse()
        lista_x.reverse()

        for i, dato in enumerate(lista_z):
            if dato < 400:
                list_temp.append(lista_x[i])
            else:
                break

        indice_fin_camion = list_temp[-1]
        

        return indice_fin_camion

    def inicioCamion(self, lista_indice : list) -> int:
        inicio_camion = lista_indice[0]
        return inicio_camion

    def finCarro(self, lista_indice : list) -> int:
        fin_carro = lista_indice[-1]
        return fin_carro

    def detectar_base_banco_full_estereo(self,banco_x : list,
                                        num_banco : int, 
                                        id_pesaje : str):
        parte_Camion = num_banco[:-2]
        ruta_directorio = self.obtener_ruta_actual()
        ruta_configuraciones = ruta_directorio+"/SETUP/configuraciones.json"
        ruta_camiones = self.cargar_datos(ruta_configuraciones,"Ruta-Camiones")
        limites_detectar_base = self.cargar_datos(ruta_configuraciones,"LIMITES-DETECTAR-BASE")
        self.path = ruta_camiones+"/"+ id_pesaje
        lista_original_Bottom_z = self.cargar_datos(self.path+'/Puntos-Bottom.json','z')
        Bottom_z = self.regenerete_list_z(lista_original_Bottom_z)
        base_full = []
        centro_banco = round( len(banco_x) / 2 )
        b4_bottom_z = Bottom_z[banco_x[0]-1:banco_x[-1]+1]
        #sum_lista = []
        for linea in b4_bottom_z:
            indice_banco = []
            cont = False
            anterior = 0
            rango = linea[limites_detectar_base[0]:limites_detectar_base[1]]

            for i, y in enumerate(rango):
                if y != 0.0:
                    #sum_lista.append(round(y))
                    if cont == True:
                        if y < limites_detectar_base[2]:
                            dif = abs(y - anterior)
                            anterior = y
                            if dif < 25:
                                indice_banco.append(limites_detectar_base[0]+i)
                            else:
                                break
                        else:
                            break
                    else:
                        anterior = y
                        cont = True
            #print(indice_banco)
            if indice_banco != []:
                base = indice_banco[-1]
                base+=1
                base_full.append(base)

        #print(base_full)

        indices_estacas, distancia_promedio = self.deteccion_estacas_banco(num_banco,
                                                                            b4_bottom_z,
                                                                            base_full)
                                        
        new_base_full = []
        #print("indices_estacas longitud",len(indices_estacas))
        if indices_estacas != []:
            for i, val in enumerate(base_full):
                if (i not in indices_estacas):
                    new_base_full.append(val)
            base_prom = self.promedio_dinamico(new_base_full,True)
            #print("Promedio de indice base con estacas",base_prom)
        else:
            #print(base_full)
            base_prom = self.promedio_dinamico(base_full,True)
            #print("Promedio de indice base sin estacas",base_prom)

        #distancia_promedio = round(sum(sum_lista) / len(sum_lista[:-2]))

        #************  Calculo de base en Metros **************
        # base_final = round (2.24501 + (-0.023968*base_prom) + (-0.000359*distancia_promedio),2)
        base_invertida = round (2.75499 + (0.023968 * base_prom) + (0.000358676 * distancia_promedio),2)
        base_final = base_invertida

        #*********** INICIO Asignar Distancias a variables globales ********

        if num_banco == 'CAMION_A':
            self.BASE_1_CAMION_INV = base_invertida
        if num_banco == 'CAMION_B':
            self.BASE_2_CAMION_INV = base_invertida
        if num_banco == 'CARRO_A':
            self.BASE_1_CARRO_INV = base_invertida
        if num_banco == 'CARRO_B':
            self.BASE_2_CARRO_INV = base_invertida


        #*********** FIN Asignar Distancias a variables globales ***********


        #**********  FIN Calculo de base en Metros ************

        self.dibujar_base_banco(int(base_prom), 
                                        banco_x[ int(centro_banco) ] +1 ,
                                        num_banco,
                                        id_pesaje,
                                        distancia_promedio,
                                        base_final)
        base_final = base_final*100
        return base_final

    def detectar_base_banco_full_estereo_bottom(self,banco_x : list,
                                        num_banco : int, 
                                        id_pesaje : str):
        parte_Camion = num_banco[:-2]
        ruta_directorio = self.obtener_ruta_actual()
        ruta_configuraciones = ruta_directorio+"/SETUP/configuraciones.json"
        ruta_camiones = self.cargar_datos(ruta_configuraciones,"Ruta-Camiones")
        limites_detectar_base = self.cargar_datos(ruta_configuraciones,"LIMITES-DETECTAR-BASE")
        self.path = ruta_camiones+"/"+ id_pesaje
        lista_original_Bottom_z = self.cargar_datos(self.path+'/Puntos-Bottom.json','z')
        Bottom_z = self.regenerete_list_z(lista_original_Bottom_z)
        base_full = []
        centro_banco = round( len(banco_x) / 2 )
        b4_bottom_z = Bottom_z[banco_x[0]-1:banco_x[-1]+1]
        #sum_lista = []
        for linea in b4_bottom_z:
            indice_banco = []
            cont = False
            anterior = 0
            rango = linea[5:26]

            for i, y in enumerate(rango):
                if y != 0.0:
                    #sum_lista.append(round(y))
                    if cont == True:
                        if y < limites_detectar_base[2]:
                            dif = abs(y - anterior)
                            anterior = y
                            if dif < 25:
                                indice_banco.append(limites_detectar_base[0]+i)
                            else:
                                break
                        else:
                            break
                    else:
                        anterior = y
                        cont = True
            #print(indice_banco)
            if indice_banco != []:
                base = indice_banco[-1]
                base+=1
                base_full.append(base)

        #print(base_full)

        indices_estacas, distancia_promedio = self.deteccion_estacas_banco(num_banco,
                                                                            b4_bottom_z,
                                                                            base_full)
                                        
        new_base_full = []
        #print("indices_estacas longitud",len(indices_estacas))
        if indices_estacas != []:
            for i, val in enumerate(base_full):
                if (i not in indices_estacas):
                    new_base_full.append(val)
            base_prom = self.promedio_dinamico(new_base_full,True)
            #print("Promedio de indice base con estacas",base_prom)
        else:
            #print(base_full)
            base_prom = self.promedio_dinamico(base_full,True)
            #print("Promedio de indice base sin estacas",base_prom)

        #*********** INICIO Asignar Distancias a variables globales ********

        if num_banco == 'CAMION_A':
            self.DISTANCIACAMIONBOTTOM_A = distancia_promedio
        if num_banco == 'CAMION_B':
            self.DISTANCIACAMIONBOTTOM_B = distancia_promedio
        if num_banco == 'CARRO_A':
            self.DISTANCIACARROBOTTOM_A = distancia_promedio
        if num_banco == 'CARRO_B':
            self.DISTANCIACARROBOTTOM_B = distancia_promedio

    def detectar_base_banco_full_estereo_right(self,banco_x : list,
                                        num_banco : int, 
                                        id_pesaje : str):
        parte_Camion = num_banco[:-2]
        ruta_directorio = self.obtener_ruta_actual()
        ruta_configuraciones = ruta_directorio+"/SETUP/configuraciones.json"
        ruta_camiones = self.cargar_datos(ruta_configuraciones,"Ruta-Camiones")
        limites_detectar_base = self.cargar_datos(ruta_configuraciones,"LIMITES-DETECTAR-BASE")
        self.path = ruta_camiones+"/"+ id_pesaje
        lista_original_Bottom_z = self.cargar_datos(self.path+'/Puntos-Right.json','z')
        Bottom_z = self.regenerete_list_z(lista_original_Bottom_z)
        base_full = []
        centro_banco = round( len(banco_x) / 2 )
        b4_bottom_z = Bottom_z[banco_x[0]-1:banco_x[-1]+1]
        #sum_lista = []
        for linea in b4_bottom_z:
            indice_banco = []
            cont = False
            anterior = 0
            rango = linea[5:26]

            for i, y in enumerate(rango):
                if y != 0.0:
                    #sum_lista.append(round(y))
                    if cont == True:
                        if y < 200:
                            dif = abs(y - anterior)
                            anterior = y
                            if dif < 25:
                                indice_banco.append(5+i)
                            else:
                                break
                        else:
                            break
                    else:
                        anterior = y
                        cont = True
            #print(indice_banco)
            if indice_banco != []:
                base = indice_banco[-1]
                base+=1
                base_full.append(base)

        #print(base_full)

        indices_estacas, distancia_promedio = self.deteccion_estacas_banco(num_banco,
                                                                            b4_bottom_z,
                                                                            base_full)

        #*********** INICIO Asignar Distancias a variables globales ********
        if num_banco == 'CAMION_A':
            self.DISTANCIACAMIONRIGHT_A = distancia_promedio
        if num_banco == 'CAMION_B':
            self.DISTANCIACAMIONRIGHT_B = distancia_promedio
        if num_banco == 'CARRO_A':
            self.DISTANCIACARRORIGHT_A = distancia_promedio
        if num_banco == 'CARRO_B':
            self.DISTANCIACARRORIGHT_B = distancia_promedio


        #*********** FIN Asignar Distancias a variables globales ***********
                                        
        # new_base_full = []
        #print("indices_estacas longitud",len(indices_estacas))
        # if indices_estacas != []:
        #     for i, val in enumerate(base_full):
        #         if (i not in indices_estacas):
        #             new_base_full.append(val)
        #     base_prom = self.promedio_dinamico(new_base_full,True)
            #print("Promedio de indice base con estacas",base_prom)
        # else:
            #print(base_full)
            # base_prom = self.promedio_dinamico(base_full,True)
            #print("Promedio de indice base sin estacas",base_prom)

        #distancia_promedio = round(sum(sum_lista) / len(sum_lista[:-2]))

        #************  Calculo de base en Metros **************
        # base_final = round(1.758672801 + (-0.019353499*base_prom) + (-0.00039313*distancia_promedio),2)
        # base_final = round (152.5727169 + (0.059947*base_prom) + (-0.135705649*distancia_promedio),2)
        # base_final = round(base_final / 100,2)
        # base_final = round (2.24501 + (-0.023968*base_prom) + (-0.000359*distancia_promedio),2)

        #**********  FIN Calculo de base en Metros ************

        # self.dibujar_base_banco_right(int(base_prom), 
        #                                 banco_x[ int(centro_banco) ] +1 ,
        #                                 num_banco,
        #                                 id_pesaje,
        #                                 distancia_promedio,
        #                                 base_final)
        # base_final = base_final*100
        base_final = 0
        return base_final

    def deteccion_estacas_banco(self, num_banco : int,
                                        banco : list,
                                        indice_base_banco :list):
                                        
        indice_estaca = []
        lista_promedios = []
        for i, linea in enumerate(banco[:len(indice_base_banco)]):
            promedio_temp = 0
            lista_temp = []
            for dato in linea[0:indice_base_banco[i]]:
                if dato != 0.0:
                    lista_temp.append(dato)
            promedio_temp = self.promedio_dinamico(lista_temp,True)
            lista_promedios.append(promedio_temp)

        promedio_banco = self.promedio_dinamico(lista_promedios,True)
        for i, linea in enumerate(lista_promedios):
            dif = abs(linea - promedio_banco)
            if dif >= 7:
                indice_estaca.append(i)
        return indice_estaca, promedio_banco

    def dibujar_base_banco(self,base : int,
                                num_foto : int,
                                num_banco : int,
                                id_pesaje : str,
                                distancia_promedio : int,
                                base_final : float):
        self.generar_carpeta_imagen_resultado_base(self.path,"Resultado_Base_banco")
        img = cv2.imread(self.path+'/Img/Camara-Bottom/'+str(num_foto)+'.jpg')
        #cv2.imwrite('{}/Img/Resultado_Base_banco/{}.jpg'.format(self.path,str(num_foto)),img)
        h,w,_ = img.shape

        if h < w:
            img = imutils.rotate_bound(img, 90)

        black_rect = np.ones(img.shape, dtype=np.uint8) * 0
        res = cv2.addWeighted(img, 0.5, black_rect, 0.5, 1.0)

        base+=1
        res = cv2.line(res,(0,base*20),(w,base*20),(255,255,255),5)
        res = cv2.line(res,(0,base*20),(w,base*20),(0,0,255),2)
        font = cv2.FONT_HERSHEY_SIMPLEX
        #prom_h = abs( ( ( (base-1) * 24) - h ) * escala / 360)

        cv2.putText(res,'Distancia = {}CM'.format(round(distancia_promedio,2)),(10,int(w/3)), font, 2,(255,255,255),5,cv2.LINE_AA)
        cv2.putText(res,'Distancia = {}CM'.format(round(distancia_promedio,2)),(10,int(w/3)), font, 2,(0,0,255),2,cv2.LINE_AA)

        cv2.putText(res,'Prom H = {}M'.format(base_final),(10,w-100), font, 2,(255,255,255),5,cv2.LINE_AA)
        cv2.putText(res,'Prom H = {}M'.format(base_final),(10,w-100), font, 2,(0,0,255),2,cv2.LINE_AA)

        # cv2.imshow('image',res)
        # cv2.waitKey(0)
        cv2.imwrite('{}/Img/Resultado_Base_banco/{}-Banco{}-Base.jpg'.format(self.path,id_pesaje,str(num_banco)),res)
        cv2.destroyAllWindows()

    def dibujar_base_banco_right(self,base : int,
                                num_foto : int,
                                num_banco : int,
                                id_pesaje : str,
                                distancia_promedio : int,
                                base_final : float):
        self.generar_carpeta_imagen_resultado_base(self.path,"Resultado_Base_banco")
        img = cv2.imread(self.path+'/Img/Camara-Right/'+str(num_foto)+'.jpg')
        #cv2.imwrite('{}/Img/Resultado_Base_banco/{}.jpg'.format(self.path,str(num_foto)),img)
        h,w,_ = img.shape

        if h < w:
            img = imutils.rotate_bound(img, 90)

        black_rect = np.ones(img.shape, dtype=np.uint8) * 0
        res = cv2.addWeighted(img, 0.5, black_rect, 0.5, 1.0)

        base+=1
        res = cv2.line(res,(0,base*20),(w,base*20),(255,255,255),5)
        res = cv2.line(res,(0,base*20),(w,base*20),(0,0,255),2)
        font = cv2.FONT_HERSHEY_SIMPLEX
        #prom_h = abs( ( ( (base-1) * 24) - h ) * escala / 360)

        cv2.putText(res,'Distancia = {}CM'.format(round(distancia_promedio,2)),(10,int(w/3)), font, 2,(255,255,255),5,cv2.LINE_AA)
        cv2.putText(res,'Distancia = {}CM'.format(round(distancia_promedio,2)),(10,int(w/3)), font, 2,(0,0,255),2,cv2.LINE_AA)

        cv2.putText(res,'Prom H = {}M'.format(base_final),(10,w-100), font, 2,(255,255,255),5,cv2.LINE_AA)
        cv2.putText(res,'Prom H = {}M'.format(base_final),(10,w-100), font, 2,(0,0,255),2,cv2.LINE_AA)

        # cv2.imshow('image',res)
        # cv2.waitKey(0)
        cv2.imwrite('{}/Img/Resultado_Base_banco/{}-Banco{}-Base-Right.jpg'.format(self.path,id_pesaje,str(num_banco)),res)
        cv2.destroyAllWindows()

    def preprocesamiento_datos_ancho(self, inicio_parte : int,
                                            fin_parte : int,
                                            ruta_camion : str,
                                            parte_camion : str):
        ruta_directorio = self.obtener_ruta_actual()
        ruta_configuraciones = ruta_directorio+"/SETUP/configuraciones.json"
        try:
            lista_original_bottom_z = self.cargar_datos(ruta_camion+'/Puntos-Bottom.json','z')
            Bottom_z = self.regenerete_list_z(lista_original_bottom_z)
            lista_original_right_z = self.cargar_datos(ruta_camion+'/Puntos-Right.json','z')
            Right_z = self.regenerete_list_z(lista_original_right_z)
            roi_matriz_lateral = self.cargar_datos(ruta_configuraciones, "ROI-MATRIZ-LATERAL")
            limite_matriz_lateral = self.cargar_datos(ruta_configuraciones, "LIMITES-DISTANCIA-MATRIZ-LATERAL")
            CONST_RPM = self.cargar_datos(ruta_configuraciones, "rango-promedio-movil-lateral") #CONSTANTE RANGO PROMEDIO MOVIL
            VDPM = self.cargar_datos(ruta_configuraciones, "validador-promedio-movil-lateral") #validador diferencia promedio movil 

            promedio_movil_bottom = 0
            promedio_movil_right = 0
            prom_distancia_right = 0
            prom_distancia_bottom = 0

            lista_distancia_bottom = []
            lista_distancia_right = []

            for i, lineas in enumerate(Bottom_z):
                if i >= inicio_parte and i <= fin_parte: 
                    
                    if i >= (inicio_parte + 4):

                        list_prom_linea_bottom = []
                        for val in Bottom_z[i-4:i]:
                            prom_temp_linea_bottom = self.promedio_dinamico(val[roi_matriz_lateral[0]
                                                                                :roi_matriz_lateral[1]],
                                                                                True)
                            list_prom_linea_bottom.append(prom_temp_linea_bottom)
                        promedio_movil_bottom = self.promedio_dinamico(list_prom_linea_bottom,True)
                        # list_prom_movil_bottom.append(promedio_movil_bottom)



                        for j, dato in enumerate(lineas):
                            # Reiniciar lista temporal para el promedio movil

                            if j >= roi_matriz_lateral[0] and j <= roi_matriz_lateral[1]:
                                if dato >= limite_matriz_lateral[1] and dato <= limite_matriz_lateral[0]:
                                    diferencia_dato_bottom = abs(promedio_movil_bottom - dato)
                                    # print("diferencia_dato_bottom",diferencia_dato_bottom)

                                    if diferencia_dato_bottom < 40:

                                        linea_graficar_bottom.append([j,dato])
                                        lista_distancia_bottom.append(dato)
                    
                    linea_graficar_bottom = []
                    if parte_camion == 'camion':
                        # print("Camion")
                        self.lista_graficar_camion_bottom.append([i,linea_graficar_bottom])
                    elif parte_camion == 'carro':
                        # print("Carro")
                        self.lista_graficar_carro_bottom.append([i,linea_graficar_bottom])
            

            for i, lineas in enumerate(Right_z):
                if i >= inicio_parte and i <= fin_parte: 
                    
                    if i >= (inicio_parte + 4):

                        list_prom_linea_right = []
                        for val in Right_z[i-4:i]:
                            prom_temp_linea_right = self.promedio_dinamico(val[roi_matriz_lateral[0]
                                                                                :roi_matriz_lateral[1]],
                                                                                True)
                            list_prom_linea_right.append(prom_temp_linea_right)
                        promedio_movil_right = self.promedio_dinamico(list_prom_linea_right,True)
                        # list_prom_movil_right.append(promedio_movil_right)



                        for j, dato in enumerate(lineas):
                            # Reiniciar lista temporal para el promedio movil

                            if j >= roi_matriz_lateral[0] and j <= roi_matriz_lateral[1]:
                                if dato >= limite_matriz_lateral[1] and dato <= limite_matriz_lateral[0]:
                                    diferencia_dato_right = abs(promedio_movil_right - dato)
                                    # print("diferencia_dato_right",diferencia_dato_right)

                                    if diferencia_dato_right < 40:

                                        linea_graficar_right.append([j,dato])
                                        lista_distancia_right.append(dato)
                    
                    linea_graficar_right = []
                    if parte_camion == 'camion':
                        # print("Camion")
                        self.lista_graficar_camion_right.append([i,linea_graficar_right])
                    elif parte_camion == 'carro':
                        # print("Carro")
                        self.lista_graficar_carro_right.append([i,linea_graficar_right])
            
            if parte_camion == 'camion':
                prom_distancia_bottom = round(self.promedio_dinamico(lista_distancia_bottom,True),2)
                self.DISTANCIA_CAMION_BOTTOM = prom_distancia_bottom
                prom_distancia_right = round(self.promedio_dinamico(lista_distancia_right,True),2)
                self.DISTANCIA_CAMION_RIGHT = prom_distancia_right
                ancho_parte = round(520 - ( prom_distancia_right + prom_distancia_bottom ),2)
                print("Ancho Calculado Camión", ancho_parte)
                print("Distancia Calculada Camión RIGHT", self.DISTANCIA_CAMION_RIGHT)
                print("Distancia Calculada Camión BOTTOM", self.DISTANCIA_CAMION_BOTTOM)
            elif parte_camion == 'carro':
                prom_distancia_bottom = round(self.promedio_dinamico(lista_distancia_bottom,True),2)
                self.DISTANCIA_CARRO_BOTTOM = prom_distancia_bottom
                prom_distancia_right = round(self.promedio_dinamico(lista_distancia_right,True),2)
                self.DISTANCIA_CARRO_RIGHT = prom_distancia_right
                ancho_parte = round(520 - ( prom_distancia_right + prom_distancia_bottom ),2)
                print("Ancho Calculado Carro", ancho_parte)
                print("Distancia Calculada Carro RIGHT", self.DISTANCIA_CARRO_RIGHT)
                print("Distancia Calculada Carro BOTTOM", self.DISTANCIA_CARRO_BOTTOM)
            
            return ancho_parte
        except Exception as e:
            ancho_parte = self.cargar_datos(ruta_configuraciones, "constante-ancho")

            return ancho_parte

    def calculo_ancho_automatico(self, bool_parte):
        ruta_directorio = self.obtener_ruta_actual()
        ruta_configuraciones = ruta_directorio+"/SETUP/configuraciones.json"
        ancho_portal = self.cargar_datos(ruta_configuraciones, "ANCHO-PORTAL")
        # Bool Parte si es camión 0 y carro 1
        print("Ancho Portal = ",ancho_portal)
        # Iniciar en 0 el ANCHO
        ancho = 0

        # 1 Si es CAMION
        if bool_parte == 0:

            # 2 Promediar distancias lado BOTTOM
            print("DISTANCIACAMIONBOTTOM_A:",self.DISTANCIACAMIONBOTTOM_A)
            print("DISTANCIACAMIONBOTTOM_B:",self.DISTANCIACAMIONBOTTOM_B)
            if self.DISTANCIACAMIONBOTTOM_A > 1 and self.DISTANCIACAMIONBOTTOM_B > 1:
                promedio_camion_bottom = round((self.DISTANCIACAMIONBOTTOM_A + self.DISTANCIACAMIONBOTTOM_B) / 2 )
            elif self.DISTANCIACAMIONBOTTOM_A == 1 and self.DISTANCIACAMIONBOTTOM_B > 1:
                promedio_camion_bottom = self.DISTANCIACAMIONBOTTOM_B
            elif self.DISTANCIACAMIONBOTTOM_A > 1 and self.DISTANCIACAMIONBOTTOM_B == 1:
                promedio_camion_bottom = self.DISTANCIACAMIONBOTTOM_A
            elif self.DISTANCIACAMIONBOTTOM_A == 1 and self.DISTANCIACAMIONBOTTOM_B == 1:
                promedio_camion_bottom = 240
            print("promedio_camion_bottom:",promedio_camion_bottom)

            # 3 promediar distancias lado RIGHT
            print("DISTANCIACAMIONRIGHT_A:",self.DISTANCIACAMIONRIGHT_A)  
            print("DISTANCIACAMIONRIGHT_B:",self.DISTANCIACAMIONRIGHT_B)
            if self.DISTANCIACAMIONRIGHT_A > 1 and self.DISTANCIACAMIONRIGHT_B > 1:
                promedio_camion_right = round((self.DISTANCIACAMIONRIGHT_A + self.DISTANCIACAMIONRIGHT_B) / 2 )
            elif self.DISTANCIACAMIONRIGHT_A == 1 and self.DISTANCIACAMIONRIGHT_B > 1:
                promedio_camion_right = self.DISTANCIACAMIONRIGHT_B
            elif self.DISTANCIACAMIONRIGHT_A > 1 and self.DISTANCIACAMIONRIGHT_B == 1:
                promedio_camion_right = self.DISTANCIACAMIONRIGHT_A
            elif self.DISTANCIACAMIONRIGHT_A == 1 and self.DISTANCIACAMIONRIGHT_B == 1:
                promedio_camion_right = 240
            print("promedio_camion_right:",promedio_camion_right)

            # 4 Calcular ANCHO con la resta de ambas 
            # distancias respecto al ancho del PORTAL
            ancho = (ancho_portal - promedio_camion_bottom) - promedio_camion_right

        # 1 Si es CARRO
        if bool_parte == 1:
            ancho = 0

            # 2 Promediar distancias lado BOTTOM
            print("DISTANCIACARROBOTTOM_A:",self.DISTANCIACARROBOTTOM_A)
            print("DISTANCIACARROBOTTOM_B:",self.DISTANCIACARROBOTTOM_B)
            if self.DISTANCIACARROBOTTOM_A > 1 and self.DISTANCIACARROBOTTOM_B > 1:
                promedio_carro_bottom = round((self.DISTANCIACARROBOTTOM_A + self.DISTANCIACARROBOTTOM_B) / 2 )
            elif self.DISTANCIACARROBOTTOM_A == 1 and self.DISTANCIACARROBOTTOM_B > 1:
                promedio_carro_bottom = self.DISTANCIACARROBOTTOM_B
            elif self.DISTANCIACARROBOTTOM_A > 1 and self.DISTANCIACARROBOTTOM_B == 1:
                promedio_carro_bottom = self.DISTANCIACARROBOTTOM_A
            elif self.DISTANCIACARROBOTTOM_A == 1 and self.DISTANCIACARROBOTTOM_B == 1:
                promedio_carro_bottom = 240
            print("promedio_carro_bottom:",promedio_carro_bottom)
            # promedio_ca

            # 3 promediar distancias lado RIGHT
            print("DISTANCIACARRORIGHT_A:",self.DISTANCIACARRORIGHT_A)
            print("DISTANCIACARRORIGHT_B:",self.DISTANCIACARRORIGHT_B)
            if self.DISTANCIACARRORIGHT_A > 1 and self.DISTANCIACARRORIGHT_B > 1:
                promedio_carro_right = round((self.DISTANCIACARRORIGHT_A + self.DISTANCIACARRORIGHT_B) / 2 )
            elif self.DISTANCIACARRORIGHT_A == 1 and self.DISTANCIACARRORIGHT_B > 1:
                promedio_carro_right = self.DISTANCIACARRORIGHT_B
            elif self.DISTANCIACARRORIGHT_A > 1 and self.DISTANCIACARRORIGHT_B == 1:
                promedio_carro_right = self.DISTANCIACARRORIGHT_A
            elif self.DISTANCIACARRORIGHT_A == 1 and self.DISTANCIACARRORIGHT_B == 1:
                promedio_carro_right = 240
            print("promedio_carro_right:",promedio_carro_right)

            # 4 Calcular ANCHO con la resta de ambas 
            # distancias respecto al ancho del PORTAL
            ancho = (ancho_portal - promedio_carro_bottom) - promedio_carro_right

        return ancho

    def generar_carpeta_imagen_resultado_base(self, ruta_camion : str, nombre_carpeta : str):
        carpeta = ruta_camion+"/Img/"+nombre_carpeta
        if not exists(carpeta):
            makedirs(carpeta)
        else:
            pass
            #print('Ya existe la carpeta se va a borrar')
            #shutil.rmtree(carpeta, ignore_errors=True)
            #makedirs(carpeta)

    def filtrar_cabina(self,lista_original_Top_z : list,
                            inicio_camion : int,
                            fin_camion : int,
                            camion_z : list,
                            camion_x : list,
                            lista_top_z : list,
                            lista_top_x : list):
        ruta_directorio = self.obtener_ruta_actual()
        ruta_configuraciones = ruta_directorio+"/SETUP/configuraciones.json"
        limites_filtrar_cabina = self.cargar_datos(ruta_configuraciones,"LIMITES-FILTRAR-CABINA")
        num_muestras_camion = len(lista_original_Top_z[inicio_camion:fin_camion])
        zona_fin_cabina = ( round(num_muestras_camion * 0.20) ) + inicio_camion
        # print( num_muestras_camion, round(num_muestras_camion * 0.20) , zona_fin_cabina )

        activador = False
        lista_valores = []
        fin_cabina = 0
        list_temp_consultar = []
        list_temp_consultar = lista_original_Top_z[inicio_camion:fin_camion]
        for i, z in enumerate(camion_z):
            #print('0',i,z)
            if z > limites_filtrar_cabina[1] and z < limites_filtrar_cabina[0]:
                #print('1',i,z)
                lista_valores.append(z)
                promedio = self.promedio_dinamico(lista_valores,activador)
                activador=True
                diferencia = abs(promedio - z)
                if diferencia < limites_filtrar_cabina[2]:
                    fin_cabina = camion_x[i]
                else:
                    fin_cabina = camion_x[i]
                    break
            elif z == 0.0 and i > 1:
                #print('2',i,z)
                limpiar_lista = []
                for n in list_temp_consultar[i][limites_filtrar_cabina[3]:limites_filtrar_cabina[4]]:
                    if n > limites_filtrar_cabina[1]:
                        limpiar_lista.append(n)
                promedio_vecinos = self.promedio_dinamico(limpiar_lista,True)
                if promedio_vecinos > limites_filtrar_cabina[0]:
                    fin_cabina = camion_x[i]
                    break
                    #print('promedio',i,z,promedio_vecinos)
                    #continue
            elif z > limites_filtrar_cabina[0] and i > 1 and z < camion_z[i-1]:
                fin_cabina = camion_x[i]
                break

        #PRUEBA
        camion_z_final = lista_top_z[zona_fin_cabina:fin_camion]
        camion_x_final = lista_top_x[zona_fin_cabina:fin_camion]

        #ORIGINAL
        # camion_z_final = lista_top_z[fin_cabina:fin_camion]
        # camion_x_final = lista_top_x[fin_cabina:fin_camion]

        #print(camion_x_final,camion_z_final)
        #print("Inicio de cabina:",inicio_camion)
        # print("Fin de cabina:",fin_cabina)
        #print(camion_z, camion_x)

        return camion_z_final, camion_x_final

    def separar_camion_carro(self,lista_original_Top_z : list,
                                camion_z_final : list,
                                camion_x_final : list,
                                ruta_camion : str,
                                inicio_camion : int,
                                fin_carro : int,
                                inicio_cabina : int):
        ruta_directorio = self.obtener_ruta_actual()
        ruta_configuraciones = ruta_directorio+"/SETUP/configuraciones.json"
        limites_separar_camion_carro = self.cargar_datos(ruta_configuraciones,"LIMITES-SEPARAR-CAMION-CARRO")

        #print("TEST")
        #Inicializamos variables
        fin_camion = 0
        inicio_carro = 0
        centro_separacion = 0
        lista_carro_x = []
        lista_camion_x = []
        conteo_adelante = 0
        conteo_atras = 0
        lista_atras = []
        lista_adelante = []

        """ print("Inicio_Camión",inicio_camion)
        print("Fin_Carro",fin_carro)
        print("Len camion z",len(camion_z_final))
        print("Len camion x",len(camion_x_final)) """

        #Se consigue l centro del camión sin cabina hasta al fin del carro
        centro_separacion = camion_x_final[ ( len(camion_x_final) // 2 ) ]
        indice_centro_separación = len(camion_x_final) // 2 
        # print("centro_separacion 1",camion_z_final[ ( len(camion_x_final) // 2 ) ], inicio_camion + (len(camion_x_final) // 2) )

        #Si la altura del centro es menor de 350 se busca el punto más cercano
        #hacia atrás y adelante para buscar el centro del camión
        if camion_z_final[ ( len(camion_x_final) // 2 ) ] < limites_separar_camion_carro[0]:
            #Agrupamos la lista a recorrer hacia atrás y adelante
            lista_atras =  camion_z_final[ 0 : indice_centro_separación + 1 ]
            # print("lista_atras",lista_atras)
            lista_adelante =  camion_z_final[indice_centro_separación : ]
            # print("lista_adelante",lista_adelante)
            lista_atras.reverse()
            
            #recorremos la lista y contar hasta encontrar una altura mayor a 350
            for val in lista_atras:
                if val > limites_separar_camion_carro[0]:
                    conteo_atras+=1
                    break
                else:
                    conteo_atras+=1

            #recorremos la lista y contar hasta encontrar una altura mayor a 350
            for val in lista_adelante:
                if val > limites_separar_camion_carro[0]:
                    conteo_adelante+=1
                    break
                else:
                    conteo_adelante+=1
            
            # Comparamos cual conteo es mas corto y así saber en que 
            # dirección esta el centro real del camión
            if conteo_atras < conteo_adelante:
                # print("Conteo Atrás", conteo_atras)
                # Si el conteo hacia atrás es menor entonces tomamos el 
                # centro_separación y restamo el conteo_atras para hacer 
                # el nuevo centro_separacion
                centro_separacion = camion_x_final[ indice_centro_separación - conteo_atras - 2 ]
                # print("centro_separacion 1",centro_separacion,indice_centro_separación - conteo_atras - 2 )
            else:
                # print("Conteo Adelante", conteo_adelante)
                # Si el conteo hacia adelante es menor entonces tomamos el 
                # centro_separación y sumamos el conteo_adelante para hacer 
                # el nuevo centro_separacion
                centro_separacion = camion_x_final[ indice_centro_separación + conteo_adelante + 2 ]
                # print("centro_separacion 1",centro_separacion,indice_centro_separación + conteo_adelante + 2)
            # print("centro_separacion > 350:",centro_separacion)
        
        """ img = cv2.imread(ruta_camion+'/Img/Camara-Top/'+str(centro_separacion)+'.jpg')
        cv2.imshow('Centro Separacion',img)
        cv2.waitKey(0)
        cv2.destroyAllWindows() """

        

        #***Preparar listas camion y carro***
        for val in camion_x_final:
            if val < centro_separacion:
                lista_camion_x.append(val)
            elif val > centro_separacion:
                lista_carro_x.append(val)
        
        """print("Lista camion",lista_camion_x)
        print("Lista carro",lista_carro_x)"""
        

        #***Encontrar inicio carro***
        for val in lista_carro_x:
            list_temp = []
            list_temp = lista_original_Top_z[val-1][limites_separar_camion_carro[2]:limites_separar_camion_carro[3]]
            list_temp.sort(reverse=True)
            if list_temp[0] < limites_separar_camion_carro[1]:
                inicio_carro = val-1
                break
            
        """ img = cv2.imread(ruta_camion+'/Img/Camara-Top/'+str(inicio_carro)+'.jpg')
        cv2.imshow('Inicio Carro',img)
        cv2.waitKey(0)
        cv2.destroyAllWindows() """


        #***Encontrar fin camión***
        for val in reversed(lista_camion_x):
            list_temp = []
            list_temp = lista_original_Top_z[val-1][limites_separar_camion_carro[2]:limites_separar_camion_carro[3]]
            list_temp.sort(reverse=True)
            if list_temp[0] < limites_separar_camion_carro[1]:
                fin_camion = val-1
                break
        
        """ img = cv2.imread(ruta_camion+'/Img/Camara-Top/'+str(fin_camion)+'.jpg')
        cv2.imshow('Fin Camion',img)
        cv2.waitKey(0)
        cv2.destroyAllWindows() """

        #Partes del camión para guardar el la BD
        self.INICIOCABINA_AP = inicio_cabina
        self.INICIOCAMION_AP = inicio_camion
        self.FINCAMION_AP = fin_camion
        self.INICIOCARRO_AP = inicio_carro
        self.FINCARRO_AP = fin_carro
        # print(fin_camion, inicio_carro)
        
        return fin_camion, inicio_carro

    def matriz_banco_full_estereo(self,lista_original_z : list,
                                    inicio_banco : int,fin_banco : int,
                                    inicio_roi : int,fin_roi : int) -> list:
        list_temp = []
        list_prom_linea = []
        linea_graficar = []
        list_prom_movil = []
        promedio_movil = 0
        promedio_linea = 0
        list_grafica = []
        list_lineas=[]
        
        ruta_directorio = self.obtener_ruta_actual()
        ruta_configuraciones = ruta_directorio+"/SETUP/configuraciones.json"
        CONST_RPM = self.cargar_datos(ruta_configuraciones, "rango-promedio-movil") #CONSTANTE RANGO PROMEDIO MOVIL
        VDPM = self.cargar_datos(ruta_configuraciones, "validador-promedio-movil") #validador diferencia promedio movil 
        #promedio_suavizado = self.promedio_dinamico(lista_suavizado,True)
        inicio_roi = self.cargar_datos(ruta_configuraciones, "ROI-MATRIZ-BANCO-FULL")[0]
        fin_roi = self.cargar_datos(ruta_configuraciones, "ROI-MATRIZ-BANCO-FULL")[1]
        limites_altura_matriz_banco = self.cargar_datos(ruta_configuraciones, "LIMITES-ALTURA-MATRIZ-BANCO")

        for i, lineas in enumerate(lista_original_z):
            linea_graficar = []
            if i >= inicio_banco and i <= fin_banco: 
                #list_temp.append(lineas[inicio_roi:fin_roi])
                #print(i) desde 0 empieza la i
                #Si la i es igual o mayor a inicio_banco + 4 entonces se inicia el promedio movil
                if i >= inicio_banco + CONST_RPM:

                    #****** Calculo de promedio movil por cada linea ******
                    list_prom_linea = []
                    for lineas_temp in lista_original_z[i-CONST_RPM:i]:
                        promedio_linea = self.promedio_dinamico(lineas_temp[inicio_roi:fin_roi],True)
                        list_prom_linea.append(promedio_linea)
                    #print(len(list_prom_linea)) Número de items es 4
                    promedio_movil = self.promedio_dinamico(list_prom_linea,True)
                    # list_prom_movil.append(promedio_movil)
                    #******* Fin Calculo de promedio movil por cada linea ******

                    #Recorremos la linea actual y filtramos los valores fuera del rango
                    #del promedio movil +/- 40cm, y valores mayores a 300cm
                    list_temp = []
                    for j, dato in enumerate(lineas[inicio_roi:fin_roi]):
                        diferencia_dato = abs(promedio_movil - dato)
                        if dato > limites_altura_matriz_banco[1]\
                             and dato < limites_altura_matriz_banco[0] and diferencia_dato < VDPM: 
                            list_temp.append(dato)
                            linea_graficar.append([j,dato])
                    list_lineas.append(list_temp)
                list_grafica.append([i,linea_graficar])  


                                    
        
        return list_lineas, list_grafica

    def matriz_banco_full_estereo_autocargante(self,lista_original_z : list,
                                                    inicio_banco : int,fin_banco : int,
                                                    inicio_roi : int,fin_roi : int) -> list:
        list_temp = []
        lista_graficar = []
        lista_suavizado = []
        
        ruta_directorio = self.obtener_ruta_actual()
        ruta_configuraciones = ruta_directorio+"/SETUP/configuraciones.json"
        inicio_roi = self.cargar_datos(ruta_configuraciones, "ROI-MATRIZ-BANCO-FULL")[0]
        fin_roi = self.cargar_datos(ruta_configuraciones, "ROI-MATRIZ-BANCO-FULL")[1]
        limites_altura_matriz_banco = self.cargar_datos(ruta_configuraciones, "LIMITES-ALTURA-MATRIZ-BANCO")
        
        
        #print(lista_original_z)
        for j, lineas in enumerate(lista_original_z):
            lista_valores = []
            linea_graficar = []
            if j >= inicio_banco and j <= fin_banco: 
                for i, dato in enumerate(lineas[inicio_roi:fin_roi]):
                    lista_suavizado.append(dato)
                    promedio_suavizado = self.promedio_dinamico(lista_suavizado,True)
                    diferencia_suavizado = abs(promedio_suavizado - dato)
                    if dato == 0.0:
                        pass    
                    if dato > limites_altura_matriz_banco[1]\
                        and dato < limites_altura_matriz_banco[0] \
                        and diferencia_suavizado < 35:
                        lista_valores.append(dato)
                        linea_graficar.append([i,dato])
                        promedio = self.promedio_dinamico(lista_valores,True)
                        diferencia = abs(promedio - dato)
                        if diferencia <= 40:
                            list_temp.append(lista_valores)
                        else:
                            break
            
                lista_graficar.append([j,linea_graficar[:-4]])
        #print(lista_suavizado)
        
        #print("Lista con garra Fltrada",list_temp)
        return list_temp, lista_graficar

    def base_banco_estereo(self, banco_x : list,num_banco : int):

        ruta_directorio = self.obtener_ruta_actual()
        ruta_configuraciones = ruta_directorio+"/SETUP/configuraciones.json"
        Constante_Base = self.cargar_datos(ruta_configuraciones,"Constante-Base")
        Base_Dinamica = self.cargar_datos(ruta_configuraciones,"Base-Dinamica")
        id_pesaje = self.PES_ID

        if Base_Dinamica == True:
            base = self.detectar_base_banco_full_estereo(banco_x,num_banco,id_pesaje)
            base_right = self.detectar_base_banco_full_estereo_right(banco_x,num_banco,id_pesaje)
            self.detectar_base_banco_full_estereo_bottom(banco_x,num_banco,id_pesaje)
        else:
            base = Constante_Base

        #print("BASE B"+str(num_banco),(base/100))

        return base

    def obtener_ruta_actual(self):
        chdir(path.dirname(__file__))
        ruta_actual = getcwd()
        return ruta_actual.replace("\\", "/")

    def get_lidar_factor_full(self,lidar_list : list, 
                                top_list : list,
                                inicio : int,
                                fin : int,
                                banco : str):
        
        grupo_lidar = []
        grupo_top = []
        #print(banco+':',lidar_list,top_list,inicio,fin)
        for i, lidar in enumerate(lidar_list):
            grupo_lidar.append(int(lidar))
            grupo_top.append(500-int(top_list[i]))
        grupo_lidar = grupo_lidar[inicio+2:fin-2]
        grupo_top = grupo_top[inicio+2:fin-2]
        prom_lidar = sum(grupo_lidar)/len(grupo_lidar)
        prom_top = sum(grupo_top)/len(grupo_top)
        factor_correccion = prom_lidar / prom_top
        #print(banco+':',factor_correccion)
        return factor_correccion

    def conexion_db(self,servidor,usuario,clave,nombre_db):
        try:
            db = mdb.connect(servidor, usuario, clave, nombre_db)
            if db:
                return db
        except Exception as e:
            self.escribirArchivoLog("Error Conexión Base de Datos: "+str(e))

    def insert_to_medicion_portal(self,query,conec_db):
        #print("Función insert_to_medicion_portal")
        try:
            cursor = conec_db.cursor()
            #self.escribirArchivoLog(cursor)
        except TypeError as f:
            self.escribirArchivoLog("Error insert datos camion: "+str(f))
        try:
            cursor.execute(query)
            conec_db.commit()
        except TypeError as g:
            self.escribirArchivoLog("Error insert datos camion: "+str(g))
        """ try:
            md_id = cursor.lastrowid
            #print("MD_ID", md_id)
        except Exception as e:
            self.escribirArchivoLog("Error insert datos camion: "+str(e))
        return md_id """

    def cerrar_db(self,conec_db):
        conec_db.close()

    def guardar_db_estereo(self):
        #===========   INSERTAR EN BASE DE DATOS   =========== 
        #Instanciar clase donde están los métodos para trabajar con MySQL
        ruta_directorio = self.obtener_ruta_actual()
        ruta_configuraciones = ruta_directorio+"/SETUP/configuraciones.json"

        #Variables conección con DB
        host_db = self.cargar_datos(ruta_configuraciones, "host_db")
        usuario_db = self.cargar_datos(ruta_configuraciones, "usuario_db")
        clave_db = self.cargar_datos(ruta_configuraciones, "clave_db")
        nombre_db = self.cargar_datos(ruta_configuraciones, "nombre_db")

        
        #Captura Datos para insertar en DB
        hoyHora = datetime.today()
        formatoHora = "%H:%M:%S"
        format_hora = hoyHora.strftime(formatoHora)
        formatoFecha = "%Y-%m-%d"
        format_fecha = hoyHora.strftime(formatoFecha)
        PES_ID = self.PES_ID
        IDMEDICION = self.IDMEDICION
        MP_FECHA = format_fecha
        MP_HORA = format_hora
        MP_PATENTE = self.MP_PATENTE
        MP_PATENTE_CARRO = self.MP_PATENTE_CARRO
        MRCAMION_AP = self.MR_CAMION_E
        MRCARRO_AP = self.MR_CARRO_E
        ALTURA_CAMION = round(self.ALTURA_CAMION_E,2)
        ALTURA_CARRO = round(self.ALTURA_CARRO_E,2)
        PROM_BASE_CAMION = round( ((self.BASE_1_CAMION_E + self.BASE_2_CAMION_E) / 2),2)
        PROM_BASE_CARRO = round( ((self.BASE_1_CARRO_E + self.BASE_2_CARRO_E) / 2),2)
        #print(MRCAMION_AP,MRCARRO_AP)
        if MRCAMION_AP == '':
            MRCAMION_AP = 0.0
        if MRCARRO_AP == '':
            MRCARRO_AP = 0.0
        TOTALMR_AP = round(float(MRCAMION_AP) + float(MRCARRO_AP),2)
        INICIOCABINA_AP = self.INICIOCABINA_AP
        INICIOCAMION_AP = self.INICIOCAMION_AP
        FINCAMION_AP = self.FINCAMION_AP
        INICIOCARRO_AP = self.INICIOCARRO_AP
        FINCARRO_AP = self.FINCARRO_AP

        #Prepara query insertar tabla medicion_portal
        # query = "INSERT INTO medicion_portal (PES_ID, MP_FECHA, MP_HORA, MP_PATENTE, \
        #     MRCAMION_AP, MRCARRO_AP, TOTALMR_AP, \
        #         INICIOCABINA_AP, INICIOCAMION_AP, FINCAMION_AP,\
        #              INICIOCARRO_AP, FINCARRO_AP, PROM_ALTURA_CAMION_AP, \
        #                  PROM_BASE_CAMION_AP, PROM_ALTURA_CARRO_AP, PROM_BASE_CARRO_AP, MP_PATENTE_CARRO) VALUES ('"+str(PES_ID)+"', '"+str(MP_FECHA)+"', '"+str(MP_HORA)+"', '"+str(MP_PATENTE)+"', '"+str(MRCAMION_AP)+"', '"+str(MRCARRO_AP)+"', '"+str(TOTALMR_AP)+"', '"+str(INICIOCABINA_AP)+"', '"+str(INICIOCAMION_AP)+"', '"+str(FINCAMION_AP)+"', '"+str(INICIOCARRO_AP)+"', '"+str(FINCARRO_AP)+"', '"+str(ALTURA_CAMION)+"', '"+str(PROM_BASE_CAMION)+"', '"+str(ALTURA_CARRO)+"', '"+str(PROM_BASE_CARRO)+"', '"+str(MP_PATENTE_CARRO)+"')"
        query = "UPDATE medicion_portal SET \
                    PES_ID = '"+str(PES_ID)+"', MP_FECHA = '"+str(MP_FECHA)+"', MP_HORA = '"+str(MP_HORA)+"',\
                    MP_PATENTE = '"+str(MP_PATENTE)+"', MRCAMION_AP = '"+str(MRCAMION_AP)+"', \
                    MRCARRO_AP = '"+str(MRCARRO_AP)+"', TOTALMR_AP = '"+str(TOTALMR_AP)+"',\
                    INICIOCABINA_AP = '"+str(INICIOCABINA_AP)+"', INICIOCAMION_AP = '"+str(INICIOCAMION_AP)+"',\
                    FINCAMION_AP = '"+str(FINCAMION_AP)+"', INICIOCARRO_AP = '"+str(INICIOCARRO_AP)+"',\
                    FINCARRO_AP = '"+str(FINCARRO_AP)+"', PROM_ALTURA_CAMION_AP = '"+str(ALTURA_CAMION)+"',\
                    PROM_BASE_CAMION_AP = '"+str(PROM_BASE_CAMION)+"', PROM_ALTURA_CARRO_AP = '"+str(ALTURA_CARRO)+"', \
                    PROM_BASE_CARRO_AP = '"+str(PROM_BASE_CARRO)+"', MP_PATENTE_CARRO = '"+str(MP_PATENTE_CARRO)+"', \
                    SINCRONIZADO = '0'\
                    WHERE MP_ID = '"+str(IDMEDICION)+"'"
                        
        # print(query)

        #Ejecutar método obtener conección con DB
        #print((host_db, usuario_db, clave_db, nombre_db))
        conec_db = self.conexion_db(host_db, usuario_db, clave_db, nombre_db)

        #Ejecutar método para insertar en la tabla medicion_portal y obtener el MD_ID
        self.insert_to_medicion_portal(query,conec_db)
        #print(md_id)

                                                        

        #Cerrar conección DB
        self.cerrar_db(conec_db)


        #===========   FIN INSERTAR EN BASE DE DATOS   =========== 

    def guardar_db_estereo_lc(self):
        #===========   INSERTAR EN BASE DE DATOS   =========== 
        #Instanciar clase donde están los métodos para trabajar con MySQL
        ruta_directorio = self.obtener_ruta_actual()
        ruta_configuraciones = ruta_directorio+"/SETUP/configuraciones.json"

        #Variables conección con DB
        host_db = self.cargar_datos(ruta_configuraciones, "host_db")
        usuario_db = self.cargar_datos(ruta_configuraciones, "usuario_db")
        clave_db = self.cargar_datos(ruta_configuraciones, "clave_db")
        nombre_db = self.cargar_datos(ruta_configuraciones, "nombre_db")

        
        #Captura Datos para insertar en DB
        hoyHora = datetime.today()
        formatoHora = "%H:%M:%S"
        format_hora = hoyHora.strftime(formatoHora)
        formatoFecha = "%Y-%m-%d"
        format_fecha = hoyHora.strftime(formatoFecha)
        PES_ID = self.PES_ID
        IDMEDICION = self.IDMEDICION
        MP_FECHA = format_fecha
        MP_HORA = format_hora
        MP_PATENTE = self.MP_PATENTE
        MP_PATENTE_CARRO = self.MP_PATENTE_CARRO
        MRCAMION_AP = self.MR_CAMION_LC
        MRCARRO_AP = self.MR_CARRO_LC
        ALTURA_CAMION = round(self.ALTURA_CAMION_LC,2)
        ALTURA_CARRO = round(self.ALTURA_CARRO_LC,2)
        PROM_BASE_CAMION = round( ((self.BASE_1_CAMION_LC + self.BASE_2_CAMION_LC) / 2),2)
        PROM_BASE_CARRO = round( ((self.BASE_1_CARRO_LC + self.BASE_2_CARRO_LC) / 2),2)
        #print(MRCAMION_AP,MRCARRO_AP)
        if MRCAMION_AP == '':
            MRCAMION_AP = 0.0
        if MRCARRO_AP == '':
            MRCARRO_AP = 0.0
        TOTALMR_AP = round(float(MRCAMION_AP) + float(MRCARRO_AP),2)
        INICIOCABINA_AP = self.INICIOCABINA_AP
        INICIOCAMION_AP = self.INICIOCAMION_AP
        FINCAMION_AP = self.FINCAMION_AP
        INICIOCARRO_AP = self.INICIOCARRO_AP
        FINCARRO_AP = self.FINCARRO_AP

        #Prepara query insertar tabla medicion_portal
        # query = "INSERT INTO medicion_portal (PES_ID, MP_FECHA, MP_HORA, MP_PATENTE, MRCAMION_AP, MRCARRO_AP, TOTALMR_AP, INICIOCABINA_AP, INICIOCAMION_AP, FINCAMION_AP, INICIOCARRO_AP, FINCARRO_AP, PROM_ALTURA_CAMION_AP, PROM_BASE_CAMION_AP, PROM_ALTURA_CARRO_AP, PROM_BASE_CARRO_AP, MP_PATENTE_CARRO) VALUES ('"+str(PES_ID)+"', '"+str(MP_FECHA)+"', '"+str(MP_HORA)+"', '"+str(MP_PATENTE)+"', '"+str(MRCAMION_AP)+"', '"+str(MRCARRO_AP)+"', '"+str(TOTALMR_AP)+"', '"+str(INICIOCABINA_AP)+"', '"+str(INICIOCAMION_AP)+"', '"+str(FINCAMION_AP)+"', '"+str(INICIOCARRO_AP)+"', '"+str(FINCARRO_AP)+"', '"+str(ALTURA_CAMION)+"', '"+str(PROM_BASE_CAMION)+"', '"+str(ALTURA_CARRO)+"', '"+str(PROM_BASE_CARRO)+"', '"+str(MP_PATENTE_CARRO)+"')"
        query = "UPDATE medicion_portal SET \
                    PES_ID = '"+str(PES_ID)+"', MP_FECHA = '"+str(MP_FECHA)+"', MP_HORA = '"+str(MP_HORA)+"',\
                    MP_PATENTE = '"+str(MP_PATENTE)+"', MRCAMION_AP = '"+str(MRCAMION_AP)+"', \
                    MRCARRO_AP = '"+str(MRCARRO_AP)+"', TOTALMR_AP = '"+str(TOTALMR_AP)+"',\
                    INICIOCABINA_AP = '"+str(INICIOCABINA_AP)+"', INICIOCAMION_AP = '"+str(INICIOCAMION_AP)+"',\
                    FINCAMION_AP = '"+str(FINCAMION_AP)+"', INICIOCARRO_AP = '"+str(INICIOCARRO_AP)+"',\
                    FINCARRO_AP = '"+str(FINCARRO_AP)+"', PROM_ALTURA_CAMION_AP = '"+str(ALTURA_CAMION)+"',\
                    PROM_BASE_CAMION_AP = '"+str(PROM_BASE_CAMION)+"', PROM_ALTURA_CARRO_AP = '"+str(ALTURA_CARRO)+"', \
                    PROM_BASE_CARRO_AP = '"+str(PROM_BASE_CARRO)+"', MP_PATENTE_CARRO = '"+str(MP_PATENTE_CARRO)+"', \
                    SINCRONIZADO = '0'\
                    WHERE MP_ID = '"+str(IDMEDICION)+"'"
        # print(query)

        #Ejecutar método obtener conección con DB
        #print((host_db, usuario_db, clave_db, nombre_db))
        conec_db = self.conexion_db(host_db, usuario_db, clave_db, nombre_db)

        #Ejecutar método para insertar en la tabla medicion_portal y obtener el MD_ID
        self.insert_to_medicion_portal(query,conec_db)
        #print(md_id)

                                                        

        #Cerrar conección DB
        self.cerrar_db(conec_db)


        #===========   FIN INSERTAR EN BASE DE DATOS   =========== 

    def guardar_db_estereo_defecto(self):
        try:
            #===========   INSERTAR EN BASE DE DATOS   =========== 
            #Instanciar clase donde están los métodos para trabajar con MySQL
            ruta_directorio = self.obtener_ruta_actual()
            ruta_configuraciones = ruta_directorio+"/SETUP/configuraciones.json"

            #Variables conección con DB
            host_db = self.cargar_datos(ruta_configuraciones, "host_db")
            usuario_db = self.cargar_datos(ruta_configuraciones, "usuario_db")
            clave_db = self.cargar_datos(ruta_configuraciones, "clave_db")
            nombre_db = self.cargar_datos(ruta_configuraciones, "nombre_db")

            
            #Captura Datos para insertar en DB
            hoyHora = datetime.today()
            formatoHora = "%H:%M:%S"
            format_hora = hoyHora.strftime(formatoHora)
            formatoFecha = "%Y-%m-%d"
            format_fecha = hoyHora.strftime(formatoFecha)
            PES_ID = self.PES_ID
            IDMEDICION = self.IDMEDICION
            MP_FECHA = format_fecha
            MP_HORA = format_hora
            MP_PATENTE = self.MP_PATENTE
            MP_PATENTE_CARRO = self.MP_PATENTE_CARRO
            MRCAMION_AP = 1
            MRCARRO_AP = 1
            ALTURA_CAMION = 0
            ALTURA_CARRO = 0
            PROM_BASE_CAMION = 0
            PROM_BASE_CARRO = 0
            if MRCAMION_AP == '':
                MRCAMION_AP = 0.0
            if MRCARRO_AP == '':
                MRCARRO_AP = 0.0
            TOTALMR_AP = 2
            self.INICIOCABINA_AP = 1
            self.INICIOCAMION_AP = 2
            self.FINCAMION_AP = 3
            self.INICIOCARRO_AP = 4
            self.FINCARRO_AP = 5
            INICIOCABINA_AP = self.INICIOCABINA_AP
            INICIOCAMION_AP = self.INICIOCAMION_AP
            FINCAMION_AP = self.FINCAMION_AP
            INICIOCARRO_AP = self.INICIOCARRO_AP
            FINCARRO_AP = self.FINCARRO_AP

            #Prepara query insertar tabla medicion_portal
            # query = "INSERT INTO medicion_portal (PES_ID, MP_FECHA, MP_HORA, MP_PATENTE, MRCAMION_AP, MRCARRO_AP, TOTALMR_AP, INICIOCABINA_AP, INICIOCAMION_AP, FINCAMION_AP, INICIOCARRO_AP, FINCARRO_AP, PROM_ALTURA_CAMION_AP, PROM_BASE_CAMION_AP, PROM_ALTURA_CARRO_AP, PROM_BASE_CARRO_AP, MP_PATENTE_CARRO) VALUES ('"+str(PES_ID)+"', '"+str(MP_FECHA)+"', '"+str(MP_HORA)+"', '"+str(MP_PATENTE)+"', '"+str(MRCAMION_AP)+"', '"+str(MRCARRO_AP)+"', '"+str(TOTALMR_AP)+"', '"+str(INICIOCABINA_AP)+"', '"+str(INICIOCAMION_AP)+"', '"+str(FINCAMION_AP)+"', '"+str(INICIOCARRO_AP)+"', '"+str(FINCARRO_AP)+"', '"+str(ALTURA_CAMION)+"', '"+str(PROM_BASE_CAMION)+"', '"+str(ALTURA_CARRO)+"', '"+str(PROM_BASE_CARRO)+"', '"+str(MP_PATENTE_CARRO)+"')"
            query = "UPDATE medicion_portal SET \
                    PES_ID = '"+str(PES_ID)+"', MP_FECHA = '"+str(MP_FECHA)+"', MP_HORA = '"+str(MP_HORA)+"',\
                    MP_PATENTE = '"+str(MP_PATENTE)+"', MRCAMION_AP = '"+str(MRCAMION_AP)+"', \
                    MRCARRO_AP = '"+str(MRCARRO_AP)+"', TOTALMR_AP = '"+str(TOTALMR_AP)+"',\
                    INICIOCABINA_AP = '"+str(INICIOCABINA_AP)+"', INICIOCAMION_AP = '"+str(INICIOCAMION_AP)+"',\
                    FINCAMION_AP = '"+str(FINCAMION_AP)+"', INICIOCARRO_AP = '"+str(INICIOCARRO_AP)+"',\
                    FINCARRO_AP = '"+str(FINCARRO_AP)+"', PROM_ALTURA_CAMION_AP = '"+str(ALTURA_CAMION)+"',\
                    PROM_BASE_CAMION_AP = '"+str(PROM_BASE_CAMION)+"', PROM_ALTURA_CARRO_AP = '"+str(ALTURA_CARRO)+"', \
                    PROM_BASE_CARRO_AP = '"+str(PROM_BASE_CARRO)+"', MP_PATENTE_CARRO = '"+str(MP_PATENTE_CARRO)+"', \
                    SINCRONIZADO = '0'\
                    WHERE MP_ID = '"+str(IDMEDICION)+"'"
            # print(query)
            #     query = "INSERT INTO medicion_portal (PES_ID, MP_FECHA, MP_HORA, MP_PATENTE, MRCAMION_AP, MRCARRO_AP, TOTALMR_AP, INICIOCABINA_AP, INICIOCAMION_AP, FINCAMION_AP, INICIOCARRO_AP, FINCARRO_AP, PROM_ALTURA_CAMION_AP, PROM_BASE_CAMION_AP, PROM_ALTURA_CARRO_AP, PROM_BASE_CARRO_AP, MP_PATENTE_CARRO) VALUES ('"+str(PES_ID)+"', '"+str(MP_FECHA)+"', '"+str(MP_HORA)+"', '"+str(MP_PATENTE)+"', '"+str(MRCAMION_AP)+"', '"+str(MRCARRO_AP)+"', '"+str(TOTALMR_AP)+"', '"+str(INICIOCABINA_AP)+"', '"+str(INICIOCAMION_AP)+"', '"+str(FINCAMION_AP)+"', '"+str(INICIOCARRO_AP)+"', '"+str(FINCARRO_AP)+"', '"+str(ALTURA_CAMION)+"', '"+str(PROM_BASE_CAMION)+"', '"+str(ALTURA_CARRO)+"', '"+str(PROM_BASE_CARRO)+"', '"+str(MP_PATENTE_CARRO)+"')"
            #     print(query)

            #Ejecutar método obtener conección con DB
            #print((host_db, usuario_db, clave_db, nombre_db))
            conec_db = self.conexion_db(host_db, usuario_db, clave_db, nombre_db)

            #Ejecutar método para insertar en la tabla medicion_portal y obtener el MD_ID
            self.insert_to_medicion_portal(query,conec_db)
            #print(md_id)

                                                            

            #Cerrar conección DB
            self.cerrar_db(conec_db)


            #===========   FIN INSERTAR EN BASE DE DATOS   ===========  
        except Exception as e:
            self.escribirArchivoLog("Error guardar_db_estereo_defecto: "+str(e))

    def get_lidar_sync_data(self, path : str) -> list:
        try:
            if exists(path):
                x=[]
                file = open(path,'r')
                lineas = file.readlines()
                file.close()

                for i, linea in enumerate(lineas):
                    if i != 0:
                        temp_list = linea.rstrip('\n').split(';')
                        #print(temp_list[1])
                        x.append(temp_list[1])
                        temp_list=[]
                return x
            else:
                return None
        except Exception as e:
            self.escribirArchivoLog("Error Leer Archivo NPesaje-SYNC: "+str(e))
    
    def crearCarpetaExportar(self):
        ruta_directorio = self.obtener_ruta_actual()
        ruta_configuraciones = ruta_directorio+"/SETUP/configuraciones.json"
        ruta_exportar_camiones = self.cargar_datos(ruta_configuraciones, 'Path-Exportar')
        path_exportar = ruta_exportar_camiones+"/"+str(self.PES_ID)

        #Crear Carpeta camión sino existe
        if not exists(path_exportar):
            makedirs(path_exportar)
        

        #Crear Carpeta Img sino existe
        if not exists(path_exportar+"/Img"):
            makedirs(path_exportar+"/Img")

        #Crear Carpeta Camara-Top sino existe
        if not exists(path_exportar+"/Img/Camara-Top"):
            makedirs(path_exportar+"/Img/Camara-Top")
        else:
            #print('Ya existe la carpeta se va a borrar')
            shutil.rmtree(path_exportar+"/Img/Camara-Top", ignore_errors=True)
            time.sleep(2)
            makedirs(path_exportar+"/Img/Camara-Top")

    def copiarArchivosExportar(self):
        try:
            ruta_directorio = self.obtener_ruta_actual()
            ruta_configuraciones = ruta_directorio+"/SETUP/configuraciones.json"
            ruta_exportar_camiones = self.cargar_datos(ruta_configuraciones, 'Path-Exportar')
            ruta_camiones = self.cargar_datos(ruta_configuraciones, 'Ruta-Camiones')

            intervalo_foto_top = self.cargar_datos(ruta_configuraciones, 'intervalo-foto-top')

            ruta_original = ruta_camiones+"/"+str(self.PES_ID)
            ruta_destino = ruta_exportar_camiones+"/"+str(self.PES_ID)

            #Copiar archivos a PC Romanero
            if exists(ruta_original+'/Puntos-Top.json'):
                shutil.copy(ruta_original+'/Puntos-Top.json', ruta_destino+'/Puntos-Top.json')
            
            if exists(ruta_original+'/Puntos-Corner.json'):
                shutil.copy(ruta_original+'/Puntos-Corner.json', ruta_destino+'/Puntos-Corner.json')
            
            if exists(ruta_original+'/Puntos-Bottom.json'):
                shutil.copy(ruta_original+'/Puntos-Bottom.json', ruta_destino+'/Puntos-Bottom.json')
            
            if exists(ruta_original+'/Puntos-Bottom.json'):
                shutil.copy(ruta_original+'/Puntos-Right.json', ruta_destino+'/Puntos-Bottom.json')
            
            if exists(ruta_original+'/Puntos-Top.txt'):
                shutil.copy(ruta_original+'/Puntos-Top.txt', ruta_destino+'/Puntos-Top.txt')
            
            if exists(ruta_original+'/Puntos-Corner.txt'):
                shutil.copy(ruta_original+'/Puntos-Corner.txt', ruta_destino+'/Puntos-Corner.txt')
            
            if exists(ruta_original+'/Puntos-Bottom.txt'):
                shutil.copy(ruta_original+'/Puntos-Bottom.txt', ruta_destino+'/Puntos-Bottom.txt')
            
            if exists(ruta_original+'/Puntos-Bottom.txt'):
                shutil.copy(ruta_original+'/Puntos-Right.txt', ruta_destino+'/Puntos-Bottom.txt')
            
            if exists(ruta_original+'/Datos-Camion.txt'):
                shutil.copy(ruta_original+'/Datos-Camion.txt', ruta_destino+'/Datos-Camion.txt')
            
            if exists(ruta_original+'/Datos-Camion.txt'):
                shutil.copy(ruta_original+'/Datos-Camion.txt', ruta_destino+'/Datos-Camion.txt')
            
            if exists(ruta_original+'/'+str(self.PES_ID)+'.txt'):
                shutil.copy(ruta_original+'/'+str(self.PES_ID)+'.txt', ruta_destino+'/'+str(self.PES_ID)+'.txt')
            
            if exists(ruta_original+'/Mediciones-'+str(self.PES_ID)+'.txt'):
                shutil.copy(ruta_original+'/Mediciones-'+str(self.PES_ID)+'.txt', ruta_destino+'/Mediciones-'+str(self.PES_ID)+'.txt')
            

            #Copiar carpeta de resultados base
            if exists(ruta_original+'/Img/Resultado_Base_banco'):
                if exists(ruta_destino+'/Img/Resultado_Base_banco'):
                    shutil.rmtree(ruta_destino+'/Img/Resultado_Base_banco', ignore_errors=True)
                shutil.copytree(ruta_original+'/Img/Resultado_Base_banco', ruta_destino+'/Img/Resultado_Base_banco')
            #Copiar carpeta de imagen graficas
            if exists(ruta_original+'/Img/Graficas'):
                if exists(ruta_destino+'/Img/Graficas'):
                    shutil.rmtree(ruta_destino+'/Img/Graficas', ignore_errors=True)
                shutil.copytree(ruta_original+'/Img/Graficas', ruta_destino+'/Img/Graficas')
            #Copiar carpeta de Imagen-Camion sino se saco foto la app del operador
            if exists(ruta_original+'/Img/Imagen-Camion'):
                if not exists(ruta_destino+'/Img/Imagen-Camion'):
                    shutil.copytree(ruta_original+'/Img/Imagen-Camion', ruta_destino+'/Img/Imagen-Camion')
            # if exists(ruta_original+'/Img/Resultado_Borde_Altura_Tradicional'):
            #     shutil.copytree(ruta_original+'/Img/Resultado_Borde_Altura_Tradicional', ruta_destino+'/Img/Resultado_Borde_Altura_Tradicional')

            #Copiar fotos camara Top para slider en PC Romanero
            self.copiarFotosSlider(ruta_original,ruta_destino,intervalo_foto_top)
        except Exception as e:
            self.escribirArchivoLog("Error exportar archivos: "+str(e))

    def copiarFotosSlider(self,ruta_original : str,ruta_destino : str,intervalo_foto_top : int):
        try:
            inicio_cabina = self.INICIOCABINA_AP
            fin_Carro = self.FINCARRO_AP

            lista_fotos = listdir(ruta_original+"/Img/Camara-Top")
            # value_slider = []
            foto = self.INICIOCABINA_AP + 1 
            while foto < self.FINCARRO_AP:
                shutil.copy(ruta_original+"/Img/Camara-Top/"+str(foto)+".jpg", 
                                ruta_destino+"/Img/Camara-Top/"+str(foto)+".jpg")
                foto+=intervalo_foto_top

            """ for foto in lista_fotos:
                nombre_foto = foto.split('.')
                numero_foto = int(nombre_foto[0])
                if numero_foto >= int(inicio_cabina) and numero_foto <= int(fin_Carro):
                    shutil.copy(ruta_original+"/Img/Camara-Top/"+str(numero_foto+1)+".jpg", 
                                ruta_destino+"/Img/Camara-Top/"+str(numero_foto+1)+".jpg") """
        except Exception as e:
            self.escribirArchivoLog("Error exportar Fotos Cámara Top: "+str(e))

    def medicion_full_estereo(self,parte_camion : str,
                                    indice_inicio_parte : int,
                                    indice_fin_parte : int,
                                    lista_original_Top_z : list,
                                    lista_top_z : list,
                                    lista_top_x : list,
                                    dict_npesaje : dict,
                                    velocidad_camion : float,
                                    ruta_camion : str):
        
        print("NORMAL")
        
        ruta_directorio = self.obtener_ruta_actual()
        ruta_configuraciones = ruta_directorio+"/SETUP/configuraciones.json"

        #Constantes
        CONSTANTE_BASE = self.cargar_datos(ruta_configuraciones, "constante-base")
        CONSTANTE_ANCHO = self.cargar_datos(ruta_configuraciones, "constante-ancho")
        limites_error_altura_base = self.cargar_datos(ruta_configuraciones, "LIMITES-ERROR-ALTURA-BASE")

        camion_z_final = lista_top_z[indice_inicio_parte:indice_fin_parte]
        camion_x_final = lista_top_x[indice_inicio_parte:indice_fin_parte]

        perdida_borde = math.ceil( len(camion_x_final) * 0.01)
        indice_inicio_parte_corregido = camion_x_final[perdida_borde]
        
        #========   Bancos  Para Full lineas   ======
        
        try:
            banco_z_full, lista_graficar = self.matriz_banco_full_estereo(lista_original_z=lista_original_Top_z,
                                                            inicio_banco=indice_inicio_parte + perdida_borde,
                                                            fin_banco=indice_fin_parte - perdida_borde,
                                                            inicio_roi=17,fin_roi=38)
        except Exception as e:
            self.escribirArchivoLog("Error matriz_banco_full_estereo(): "+str(e))
        #========   FIN Bancos  Para Full lineas   ======
        
        #============   Calculo Promedio de alturas de multiples lineas   ============
        try:
            prom_h_full_con_base = self.promedio_alturas_bancos_full_estereo(banco_z_full)
        except Exception as e:
            self.escribirArchivoLog("Error promedio_alturas_bancos_full_estereo(): "+str(e))
        #print("promedio de altura CM===",parte_camion,prom_h_full_con_base)
        prom_h_full_con_base = prom_h_full_con_base / 100
        #print("promedio de altura MTS===",parte_camion,prom_h_full_con_base)
        #============   FIN Calculo Promedio de alturas de multiples lineas   ============
        
        #============   Retornar altura portal al promedio de altura   ============
        # Esta sección se devuelve la altura del portal al promedio de altura para
        # tener el valor no de la cima de la madera al suelo sino de la cima al techo 
        # del portal. En caso de volver al método anterior se debe comentar esta sección
        # y cambiar la variable que retorna el calculo de la base.
        try:
            # print("Altura antes de INV:",prom_h_full_con_base)
            altura_portal = self.cargar_datos(ruta_configuraciones, "ALTURA-PORTAL")
            altura_portal = altura_portal / 100
            prom_h_full_con_base = altura_portal - prom_h_full_con_base
        except Exception as e:
            self.escribirArchivoLog("Error retornar altura portal: "+str(e))
        #============   FIN Retornar altura portal al promedio de altura   ============

        grupo_x_full = lista_top_x[indice_inicio_parte:indice_fin_parte]
        perdida_borde = math.ceil( len(grupo_x_full) * 0.05 )
        porcentaje_grupo = math.ceil( len(grupo_x_full) * 0.20 )

        grupo_x_1 = lista_top_x[indice_inicio_parte + perdida_borde : indice_inicio_parte + perdida_borde + porcentaje_grupo]
        grupo_x_2 = lista_top_x[indice_fin_parte - perdida_borde - porcentaje_grupo : indice_fin_parte - perdida_borde]
        

        #===================   Calculo base a una linea   ==================
        base_grupo1=0
        base_grupo2=0
        try:
            base_grupo1 = self.base_banco_estereo(grupo_x_1,parte_camion+"_A")
        except Exception as e:
            base_grupo1 = CONSTANTE_BASE
            self.escribirArchivoLog("Error base_banco_estereo() A: "+str(e))
        try:
            base_grupo2 = self.base_banco_estereo(grupo_x_2,parte_camion+"_B")
        except Exception as e:
            base_grupo2 = CONSTANTE_BASE
            self.escribirArchivoLog("Error base_banco_estereo() B: "+str(e))

        base_grupo1 = base_grupo1 / 100
        base_grupo2 = base_grupo2 / 100
        #BASE NORMAL
        # if base_grupo1 > limites_error_altura_base[0] or base_grupo1 < limites_error_altura_base[1]:
        #     base_grupo1 = CONSTANTE_BASE
        # if base_grupo2 > limites_error_altura_base[0] or base_grupo2 < limites_error_altura_base[1]:
        #     base_grupo2 = CONSTANTE_BASE
        #BASE INV
        print("base_grupo1:",base_grupo1)
        print("base_grupo2:",base_grupo2)
        print("limites_error_altura_base[0]:",limites_error_altura_base[0],
                "limites_error_altura_base[1]:",limites_error_altura_base[1])
        if base_grupo1 < limites_error_altura_base[0] or base_grupo1 > limites_error_altura_base[1]:
            base_grupo1 = CONSTANTE_BASE
        if base_grupo2 < limites_error_altura_base[0] or base_grupo2 > limites_error_altura_base[1]:
            base_grupo2 = CONSTANTE_BASE
        base_promedio_parte = ( base_grupo1 + base_grupo2 ) / 2
        
        #print(base_grupo1,base_grupo2)
        #self.Base_b1 = base_b1 / 100
        #===================   FIN Calculo base a una linea   ==================

        #===================   Calculo Altura de banco sin base   ==================
        # print("Base  INV:",base_promedio_parte ,"Altura INV:", prom_h_full_con_base)
        # Método base normal
        # prom_h_full = prom_h_full_con_base - base_promedio_parte

        # Método base inversa
        prom_h_full = base_promedio_parte - prom_h_full_con_base
        # print("Altura banco:",prom_h_full)

        if prom_h_full < 1:
            prom_h_full = 1
        if prom_h_full > 3:
            prom_h_full = 3

        # Método base normal
        # prom_h_full_lc = prom_h_full_con_base - CONSTANTE_BASE

        # Método base inversa
        CONSTANTE_BASE_INV = altura_portal - CONSTANTE_BASE
        prom_h_full_lc = prom_h_full_con_base - CONSTANTE_BASE_INV

        if prom_h_full_lc < 1:
            prom_h_full_lc = 1
        if prom_h_full_lc > 3:
            prom_h_full_lc = 3
        #===================   FIN Calculo Altura de banco sin base   ==================
        
        #============   Factor de Correción a los promedio de altura   ============
        
        aplica_factor = self.cargar_datos(ruta_configuraciones, "Aplica_Factor")
        factor_correcion = self.cargar_datos(ruta_configuraciones, "Factor_Correccion")

        try:
            sync_lidar = self.get_lidar_sync_data(self.path+"/"+str(dict_npesaje['IDPESAJE'])+"-sync.txt")
        except Exception as e:
            self.escribirArchivoLog("Error archivo sync_lidar: "+str(e))

        if aplica_factor:
            if sync_lidar != None:
                self.factor_1 = self.get_lidar_factor_full(sync_lidar,lista_top_z,int(indice_inicio_parte),int(indice_fin_parte),parte_camion)
            prom_h_full = round(prom_h_full * (factor_correcion), 2)
            prom_h_full_lc = round(prom_h_full_lc * (factor_correcion), 2)
            #self.altura_b1 = prom_h_full
        #============   FIN Factor de Correción a los promedio de altura   ============

        #=============   Obtener Largo de trozo y Ancho de parte   ============

        if 'LARGOCAMION' in dict_npesaje:
            largo_camion_dict = 'LARGOCAMION'
        else:
            largo_camion_dict = 'LARGO'

        largo_camion = float(dict_npesaje[largo_camion_dict])
        #print("Largo Camion",largo_camion)
        if largo_camion > 100:
            largo_camion = round(largo_camion / 100,2)
        self.LARGOCAMION = largo_camion

        #************** INICIO CALCULO ANCHO CAMION ************
        # Para calculo_ancho_automatico(bool_parte) el argumento
        # bool_parte donde 0 es el valor para camiones y 1 el valor
        # para el carro.

        # try:
        #     # Ejecutar Función Calculo Ancho Automático
        #     # ancho_camion = self.calculo_ancho_automatico(0)
        #     ancho_camion_calculado = self.calculo_ancho_automatico(0)
        #     self.ANCHOCAMIONCALCULADO = ancho_camion_calculado
        #     print("ancho_camion_calculado:",ancho_camion_calculado)
        # except Exception as e:
        #     # Si falla el calculo automático se usa el valor de npesaje
        #     pass
        if parte_camion == 'CAMION':
            try:
                # datos_ancho_camion_preprocesados = 
                ancho_camion_calculado = self.preprocesamiento_datos_ancho(indice_inicio_parte + perdida_borde,
                                                        indice_fin_parte - perdida_borde,
                                                        ruta_camion,
                                                        'camion')
                self.ANCHO_CAMION_CALCULADO = ancho_camion_calculado
            except Exception as e:
                pass


        ancho_camion = float(dict_npesaje['ANCHOCAMION'])

        if ancho_camion > 100:
            ancho_camion = round(ancho_camion / 100,2)
            
        # print("ancho_camion:",ancho_camion)
        self.ANCHOCAMION = ancho_camion
        #************** FIN CALCULO ANCHO CAMION ***************

        
        
        if 'LARGOCARRO' in dict_npesaje:
            largo_carro_dict = 'LARGOCARRO'
        else:
            largo_carro_dict = 'LARGO'

        largo_carro = float(dict_npesaje[largo_carro_dict])
        if largo_carro > 100:
            largo_carro = round(largo_carro / 100,2)
        self.LARGOCARRO = largo_carro

        #************** INICIO CALCULO ANCHO CARRO ************
        # Para calculo_ancho_automatico(bool_parte) el argumento
        # bool_parte donde 0 es el valor para camiones y 1 el valor
        # para el carro.

        # try:
        #     # Ejecutar Función Calculo Ancho Automático
        #     # ancho_carro = self.calculo_ancho_automatico(1)
        #     ancho_carro_calculado = self.calculo_ancho_automatico(1)
        #     self.ANCHOCARROCALCULADO = ancho_carro_calculado
        #     print("ancho_carro_calculado:",ancho_carro_calculado)
        # except Exception as e:
        #     # Si falla el calculo automático se usa el valor de npesaje
        #     pass
        if parte_camion == 'CARRO':
            try:
                # datos_ancho_camion_preprocesados = 
                ancho_carro_calculado = self.preprocesamiento_datos_ancho(indice_inicio_parte + perdida_borde,
                                                        indice_fin_parte - perdida_borde,
                                                        ruta_camion,
                                                        'carro')
                self.ANCHO_CARRO_CALCULADO = ancho_carro_calculado
            except Exception as e:
                pass
        
        ancho_carro = float(dict_npesaje['ANCHOCARRO'])

        if ancho_carro > 100:
            ancho_carro = round(ancho_carro / 100,2)
            
        # print("ancho_carro:",ancho_carro)
        self.ANCHOCARRO = ancho_carro
        #************** FIN CALCULO ANCHO CARRO ***************

        

        n_bancos_camion = dict_npesaje['BANCOSCAMION']
        n_bancos_carro = dict_npesaje['BANCOSCARRO']

        #=============   FIN Obtener Largo de trozo y Ancho de parte   ============
        

        if parte_camion == 'CAMION':
            if True: # largo_camion != '' or n_bancos_camion != ''

                #============   Calculo de Metro Ruma Estereo   ===========
                # 1-- Consultar Largo de trozo y número de bancos
                if (n_bancos_camion != '' or n_bancos_camion != 'A-0') and largo_camion != '':
                    n_bancos = n_bancos_camion.split('-')
                    distancia_parte_e = float(largo_camion) * int(n_bancos[1])
                else:
                    distancia_parte_e = self.obtener_distancia_parte(indice_inicio_parte,
                                                                        indice_fin_parte,
                                                                        ruta_camion,
                                                                        velocidad_camion)
                
                                                                        
                # 2-- Consultar ancho de camión
                if ancho_camion == '' or ancho_camion == '0':
                    ancho_camion = CONSTANTE_ANCHO

                try:
                    mtr_ruma_e = self.calculo_metro_ruma_estereo(prom_h_full,float(ancho_camion),distancia_parte_e)
                except Exception as e:
                    self.escribirArchivoLog("Error Calculo MR estéreo(): "+str(e))
                #============   FIN Calculo de Metro Ruma Estereo   ===========

                #============   Calculo de Metro Ruma Largo Calculado   ===========
                #=============   Obtener Largo de trozo Automatico en función a la velocidad del camión   ============
                try:
                    distancia_parte_lc = self.obtener_distancia_parte(indice_inicio_parte,
                                                                        indice_fin_parte,
                                                                        ruta_camion,
                                                                        velocidad_camion)
                except Exception as e:
                    self.escribirArchivoLog("Error obtener_distancia_parte(): "+str(e))

                try:
                    mtr_ruma_lc = self.calculo_metro_ruma_estereo(prom_h_full_lc,float(CONSTANTE_ANCHO),distancia_parte_lc)
                except Exception as e:
                    self.escribirArchivoLog("Error Calculo MR LC: "+str(e))
                #============   FIN Calculo de Metro Ruma Largo Calculado   ===========

                #============   Guardar variables globales   ===========   
                #Datos Medición estereo con largo suministrado
                self.ALTURA_CAMION_E = prom_h_full
                self.LARGO_CAMION_E = distancia_parte_e
                self.BASE_1_CAMION_E = base_grupo1
                self.BASE_2_CAMION_E = base_grupo2
                self.ANCHO_CAMION_E = ancho_camion
                self.MR_CAMION_E = mtr_ruma_e

                #Datos Medición con largo calculado
                self.VELOCIDAD_PROMEDIO_TOTAL = velocidad_camion
                self.ALTURA_CAMION_LC = prom_h_full_lc
                self.BASE_1_CAMION_LC = CONSTANTE_BASE
                self.BASE_2_CAMION_LC = CONSTANTE_BASE
                self.ANCHO_CAMION_LC = CONSTANTE_ANCHO
                self.LARGO_CAMION_LC = distancia_parte_lc
                self.MR_CAMION_LC = mtr_ruma_lc
                
                # print("**** DATOS CAMION ****")
                # print("Promedio de Altura: ",prom_h_full)
                # print("Base 1: ",base_grupo1)
                # print("Base 2: ",base_grupo2)
                # print("Ancho Camión: ",ancho_camion)
                # print("MR E: ",mtr_ruma_e)
                # print("Promedio de Altura LC: ",prom_h_full_lc)
                # print("Base CONSTANTE: ",CONSTANTE_BASE)
                # print("Ancho Camión CONSTANTE: ",CONSTANTE_ANCHO)
                # print("Largo Trozo Calculado: ",distancia_parte_lc)
                # print("MR LC: ",mtr_ruma_lc)
                #============   FIN Guardar variables globales   ===========
            else:
                #=============   Obtener Largo de trozo Automatico en función a la velocidad del camión   ============
                try:
                    distancia_parte_lc = self.obtener_distancia_parte(indice_inicio_parte,
                                                                indice_fin_parte,
                                                                ruta_camion,
                                                                velocidad_camion)
                except Exception as e:
                    self.escribirArchivoLog("Error obtener_distancia_parte(): "+str(e))

                try:
                    mtr_ruma_lc = self.calculo_metro_ruma_estereo(prom_h_full_lc,float(CONSTANTE_ANCHO),distancia_parte_lc)
                except Exception as e:
                    self.escribirArchivoLog("Error Calculo MR LC: "+str(e))
                #=============   FIN Obtener Largo de trozo Automatico en función a la velocidad del camión   ===========

                #============   Guardar variables globales   ===========   
                #Datos Medición con largo calculado
                self.VELOCIDAD_PROMEDIO_TOTAL = velocidad_camion
                self.ALTURA_CAMION_LC = prom_h_full_lc
                self.BASE_1_CAMION_LC = CONSTANTE_BASE
                self.BASE_2_CAMION_LC = CONSTANTE_BASE
                self.ANCHO_CAMION_LC = CONSTANTE_ANCHO
                self.LARGO_CAMION_LC = distancia_parte_lc
                self.MR_CAMION_LC = mtr_ruma_lc

                # print("**** DATOS CAMION ****")
                # print("Promedio de Altura LC: ",prom_h_full_lc)
                # print("Base CONSTANTE: ",CONSTANTE_BASE)
                # print("Ancho Camión CONSTANTE: ",CONSTANTE_ANCHO)
                # print("Largo Trozo Calculado: ",distancia_parte_lc)
                # print("MR LC: ",mtr_ruma_lc)
                #============   FIN Guardar variables globales   ===========
        elif parte_camion == 'CARRO':
            if largo_carro != '' or n_bancos_carro != '':

                #============   Calculo de Metro Ruma Estereo   ===========
                # 1-- Consultar Largo de trozo y número de bancos
                if (n_bancos_carro != '' or n_bancos_carro != 'A-0') and largo_carro != '':
                    n_bancos = n_bancos_carro.split('-')
                    distancia_parte_e = float(largo_carro) * int(n_bancos[1])
                else:
                    distancia_parte_e = self.obtener_distancia_parte(indice_inicio_parte,
                                                                indice_fin_parte,
                                                                ruta_camion,
                                                                velocidad_camion)
                
                                                    
                
                                                                        
                # 2-- Consultar ancho de camión
                if ancho_carro == '' or ancho_carro == '0':
                    ancho_carro = CONSTANTE_ANCHO

                try:
                    mtr_ruma_e = self.calculo_metro_ruma_estereo(prom_h_full,float(ancho_carro),distancia_parte_e)
                except Exception as e:
                    self.escribirArchivoLog("Error Calculo MR estéreo: "+str(e))
                #============   FIN Calculo de Metro Ruma Estereo   ===========

                #============   Calculo de Metro Ruma Largo Calculado   ===========
                #=============   Obtener Largo de trozo Automatico en función a la velocidad del camión   ============
                try:
                    distancia_parte_lc = self.obtener_distancia_parte(indice_inicio_parte,
                                                                indice_fin_parte,
                                                                ruta_camion,
                                                                velocidad_camion)
                except Exception as e:
                    self.escribirArchivoLog("Error obtener_distancia_parte(): "+str(e))
                
                try:
                    mtr_ruma_lc = self.calculo_metro_ruma_estereo(prom_h_full_lc,float(CONSTANTE_ANCHO),distancia_parte_lc)
                except Exception as e:
                    self.escribirArchivoLog("Error Calculo MR LC: "+str(e))
                #============   FIN Calculo de Metro Ruma Largo Calculado   ===========

                #============   Guardar variables globales   ===========
                #Datos Medición estereo con largo suministrado
                self.ALTURA_CARRO_E = prom_h_full
                self.LARGO_CARRO_E = distancia_parte_e
                self.BASE_1_CARRO_E = base_grupo1
                self.BASE_2_CARRO_E = base_grupo2
                self.ANCHO_CARRO_E = ancho_carro
                self.MR_CARRO_E = mtr_ruma_e

                #Datos Medición con largo calculado
                self.ALTURA_CARRO_LC = prom_h_full_lc
                self.BASE_1_CARRO_LC = CONSTANTE_BASE
                self.BASE_2_CARRO_LC = CONSTANTE_BASE
                self.ANCHO_CARRO_LC = CONSTANTE_ANCHO
                self.LARGO_CARRO_LC = distancia_parte_lc
                self.MR_CARRO_LC = mtr_ruma_lc
                
                # print("**** DATOS CARRO ****")
                # print("Promedio de Altura: ",prom_h_full)
                # print("Base 1: ",base_grupo1)
                # print("Base 2: ",base_grupo2)
                # print("Ancho Camión: ",ancho_carro)
                # print("MR E: ",mtr_ruma_e)
                # print("Promedio de Altura LC: ",prom_h_full_lc)
                # print("Base CONSTANTE: ",CONSTANTE_BASE)
                # print("Ancho Camión CONSTANTE: ",CONSTANTE_ANCHO)
                # print("Largo Trozo Calculado: ",distancia_parte_lc)
                # print("MR LC: ",mtr_ruma_lc)
                #============   FIN Guardar variables globales   ===========
                
            else:
                #============   Calculo de Metro Ruma Largo Calculado   ===========
                #=============   Obtener Largo de trozo Automatico en función a la velocidad del camión   ============
                try:
                    distancia_parte_lc = self.obtener_distancia_parte(indice_inicio_parte,
                                                                indice_fin_parte,
                                                                ruta_camion,
                                                                velocidad_camion)
                except Exception as e:
                    self.escribirArchivoLog("Error obtener_distancia_parte(): "+str(e))
                
                try:
                    mtr_ruma_lc = self.calculo_metro_ruma_estereo(prom_h_full_lc,float(CONSTANTE_ANCHO),distancia_parte_lc)
                except Exception as e:
                    self.escribirArchivoLog("Error Calculo MR LC: "+str(e))
                #=============   FIN Obtener Largo de trozo Automatico en función a la velocidad del camión   ===========
                #============   FIN Calculo de Metro Ruma Largo Calculado   ===========

                #============   Guardar variables globales   ===========

                #Datos Medición con largo calculado
                self.ALTURA_CARRO_LC = prom_h_full_lc
                self.BASE_1_CARRO_LC = CONSTANTE_BASE
                self.BASE_2_CARRO_LC = CONSTANTE_BASE
                self.ANCHO_CARRO_LC = CONSTANTE_ANCHO
                self.LARGO_CARRO_LC = distancia_parte_lc
                self.MR_CARRO_LC = mtr_ruma_lc
                
                # print("**** DATOS CARRO ****")
                # print("Promedio de Altura LC: ",prom_h_full_lc)
                # print("Base CONSTANTE: ",CONSTANTE_BASE)
                # print("Ancho Camión CONSTANTE: ",CONSTANTE_ANCHO)
                # print("Largo Trozo Calculado: ",distancia_parte_lc)
                # print("MR LC: ",mtr_ruma_lc)
                #============   FIN Guardar variables globales   ===========

        return lista_graficar

    def medicion_full_estereo_autocargante(self,parte_camion : str,
                                                indice_inicio_parte : int,
                                                indice_fin_parte : int,
                                                lista_original_Top_z : list,
                                                lista_top_z : list,
                                                lista_top_x : list,
                                                dict_npesaje : dict,
                                                velocidad_camion : float,
                                                ruta_camion : str):

        print("AUTOCARGANTE")

        ruta_directorio = self.obtener_ruta_actual()
        ruta_configuraciones = ruta_directorio+"/SETUP/configuraciones.json"

        #Constantes
        CONSTANTE_BASE = self.cargar_datos(ruta_configuraciones, "constante-base")
        CONSTANTE_ANCHO = self.cargar_datos(ruta_configuraciones, "constante-ancho")
        limites_error_altura_base = self.cargar_datos(ruta_configuraciones, "LIMITES-ERROR-ALTURA-BASE")

        camion_z_final = lista_top_z[indice_inicio_parte:indice_fin_parte]
        camion_x_final = lista_top_x[indice_inicio_parte:indice_fin_parte]

        grupo_x_full = lista_top_x[indice_inicio_parte:indice_fin_parte]
        perdida_borde = math.ceil( len(grupo_x_full) * 0.15 )
        porcentaje_grupo = math.ceil( len(grupo_x_full) * 0.20 )
        #========   Bancos  Para Full lineas   ======
        try:
            banco_z_full, lista_graficar = self.matriz_banco_full_estereo_autocargante(lista_original_z=lista_original_Top_z,
                                                                                    inicio_banco=indice_inicio_parte + perdida_borde,
                                                                                    fin_banco=indice_fin_parte -perdida_borde,
                                                                                    inicio_roi=17,fin_roi=38)
        except Exception as e:
            self.escribirArchivoLog("Error matriz_banco_full_estereo_autocargante(): "+str(e))
        #========   FIN Bancos  Para Full lineas   ======
        
        #print("=====> 1")
        #print(banco_z_full,lista_graficar)
        
        #============   Calculo Promedio de alturas de multiples lineas   ============
        try:
            prom_h_full_con_base = self.promedio_alturas_bancos_full_estereo(banco_z_full)
        except Exception as e:
            self.escribirArchivoLog("Error promedio_alturas_bancos_full_estereo(): "+str(e))
        #print("promedio de altura CM===",parte_camion,prom_h_full_con_base)
        prom_h_full_con_base = prom_h_full_con_base / 100
        #print("promedio de altura MTS===",parte_camion,prom_h_full_con_base)
        #============   FIN Calculo Promedio de alturas de multiples lineas   ============
        
        #============   Retornar altura portal al promedio de altura   ============
        # Esta sección se devuelve la altura del portal al promedio de altura para
        # tener el valor no de la cima de la madera al suelo sino de la cima al techo 
        # del portal. En caso de volver al método anterior se debe comentar esta sección
        # y cambiar la variable que retorna el calculo de la base.
        try:
            altura_portal = self.cargar_datos(ruta_configuraciones, "ALTURA-PORTAL")
            altura_portal = altura_portal / 100
            prom_h_full_con_base = altura_portal - prom_h_full_con_base
        except Exception as e:
            self.escribirArchivoLog("Error retornar altura portal: "+str(e))
        #============   FIN Retornar altura portal al promedio de altura   ============

        grupo_x_1 = lista_top_x[indice_inicio_parte + perdida_borde : indice_inicio_parte + perdida_borde + porcentaje_grupo]
        grupo_x_2 = lista_top_x[indice_fin_parte - perdida_borde - porcentaje_grupo : indice_fin_parte - perdida_borde]
        

        #===================   Calculo base a una linea   ==================
        try:
            base_grupo1 = self.base_banco_estereo(grupo_x_1,parte_camion+"_A")
        except Exception as e:
            base_grupo1 = CONSTANTE_BASE
            self.escribirArchivoLog("Error base_banco_estereo() A: "+str(e))
        try:
            base_grupo2 = self.base_banco_estereo(grupo_x_2,parte_camion+"_B")
        except Exception as e:
            base_grupo2 = CONSTANTE_BASE
            self.escribirArchivoLog("Error base_banco_estereo() B: "+str(e))

        
        base_grupo1 = base_grupo1 / 100
        base_grupo2 = base_grupo2 / 100
        #BASE NORMAL
        # if base_grupo1 > limites_error_altura_base[0] or base_grupo1 < limites_error_altura_base[1]:
        #     base_grupo1 = CONSTANTE_BASE
        # if base_grupo2 > limites_error_altura_base[0] or base_grupo2 < limites_error_altura_base[1]:
        #     base_grupo2 = CONSTANTE_BASE
        #BASE INV
        if base_grupo1 < limites_error_altura_base[0] or base_grupo1 > limites_error_altura_base[1]:
            base_grupo1 = CONSTANTE_BASE
        if base_grupo2 < limites_error_altura_base[0] or base_grupo2 > limites_error_altura_base[1]:
            base_grupo2 = CONSTANTE_BASE
        base_promedio_parte = ( base_grupo1 + base_grupo2 ) / 2
        
        #print(base_grupo1,base_grupo2)
        #self.Base_b1 = base_b1 / 100
        #===================   FIN Calculo base a una linea   ==================

        #===================   Calculo Altura de banco sin base   ==================

        # Método base normal
        # prom_h_full = prom_h_full_con_base - base_promedio_parte

        # Método base inversa
        prom_h_full = base_promedio_parte - prom_h_full_con_base

        if prom_h_full < 1:
            prom_h_full = 1
        if prom_h_full > 3:
            prom_h_full = 3

        # Método base normal
        # prom_h_full_lc = prom_h_full_con_base - CONSTANTE_BASE

        # Método base inversa
        CONSTANTE_BASE_INV = altura_portal - CONSTANTE_BASE
        prom_h_full_lc = prom_h_full_con_base - CONSTANTE_BASE_INV

        if prom_h_full_lc < 1:
            prom_h_full_lc = 1
        if prom_h_full_lc > 3:
            prom_h_full_lc = 3
        # #===================   FIN Calculo Altura de banco sin base   ==================

        # #===================   Calculo Altura de banco sin base   ==================
        # prom_h_full = round(prom_h_full_con_base - base_promedio_parte,2)        
        # if prom_h_full < 1:
        #     prom_h_full = 1
        # if prom_h_full > 3:
        #     prom_h_full = 3

        # prom_h_full_lc = round(prom_h_full_con_base - CONSTANTE_BASE,2)
        # if prom_h_full_lc < 1:
        #     prom_h_full_lc = 1
        # if prom_h_full_lc > 3:
        #     prom_h_full_lc = 3
        # #===================   FIN Calculo Altura de banco sin base   ==================
        
        #============   Factor de Correción a los promedio de altura   ============
        aplica_factor = self.cargar_datos(ruta_configuraciones, "Aplica_Factor")
        factor_correcion = self.cargar_datos(ruta_configuraciones, "Factor_Correccion")
        sync_lidar = self.get_lidar_sync_data(self.path+"/"+str(dict_npesaje['IDPESAJE'])+"-sync.txt")
        if aplica_factor:
            if sync_lidar != None:
                self.factor_1 = self.get_lidar_factor_full(sync_lidar,lista_top_z,int(indice_inicio_parte),int(indice_fin_parte),'CAMION')
            prom_h_full_lc = round(prom_h_full_lc * (factor_correcion), 2)
            prom_h_full = round(prom_h_full * (factor_correcion), 2)
            #self.altura_b1 = prom_h_full
        #============   FIN Factor de Correción a los promedio de altura   ============

        #=============   Obtener Largo de trozo y Ancho de parte   ============        
        if 'LARGOCAMION' in dict_npesaje:
            largo_camion_dict = 'LARGOCAMION'
        else:
            largo_camion_dict = 'LARGO'

        largo_camion = float(dict_npesaje[largo_camion_dict])
        #print("Largo Camion",largo_camion)
        if largo_camion > 100:
            largo_camion = round(largo_camion / 100,2)
        self.LARGOCAMION = largo_camion
        
        #************** INICIO CALCULO ANCHO CAMION ************
        # Para calculo_ancho_automatico(bool_parte) el argumento
        # bool_parte donde 0 es el valor para camiones y 1 el valor
        # para el carro.

        # try:
        #     # Ejecutar Función Calculo Ancho Automático
        #     # ancho_camion = self.calculo_ancho_automatico(0)
        #     ancho_camion_calculado = self.calculo_ancho_automatico(0)
        #     self.ANCHOCAMIONCALCULADO = ancho_camion_calculado
        #     print("ancho_camion_calculado:",ancho_camion_calculado)
        # except Exception as e:
        #     # Si falla el calculo automático se usa el valor de npesaje
        #     pass
        if parte_camion == 'CAMION':
            try:
                # datos_ancho_camion_preprocesados = 
                ancho_camion_calculado = self.preprocesamiento_datos_ancho(indice_inicio_parte + perdida_borde,
                                                        indice_fin_parte - perdida_borde,
                                                        ruta_camion,
                                                        'camion')
                self.ANCHO_CAMION_CALCULADO = ancho_camion_calculado
            except Exception as e:
                pass


        ancho_camion = float(dict_npesaje['ANCHOCAMION'])

        if ancho_camion > 100:
            ancho_camion = round(ancho_camion / 100,2)
            
        # print("ancho_camion:",ancho_camion)
        self.ANCHOCAMION = ancho_camion
        #************** FIN CALCULO ANCHO CAMION ***************

        n_bancos_camion = dict_npesaje['BANCOSCAMION']

        #=============   FIN Obtener Largo de trozo y Ancho de parte   ============

        if parte_camion == 'CAMION':

            # Iniciamos variable
            distancia_parte_lc = 0
            if largo_camion != '' or n_bancos_camion != '':

                #============   Calculo de Metro Ruma Estereo   ===========
                # 1-- Consultar Largo de trozo y número de bancos
                if (n_bancos_camion != '' or n_bancos_camion != 'A-0') and largo_camion != '':
                    n_bancos = n_bancos_camion.split('-')
                    distancia_parte_e = float(largo_camion) * int(n_bancos[1])
                else:
                    distancia_parte_e = self.obtener_distancia_parte(indice_inicio_parte,
                                                                indice_fin_parte,
                                                                ruta_camion,
                                                                velocidad_camion)
                                                                        
                # 2-- Consultar ancho de camión
                if ancho_camion == '' or ancho_camion == '0':
                    ancho_camion = CONSTANTE_ANCHO

                try:
                    mtr_ruma_e = self.calculo_metro_ruma_estereo(prom_h_full,float(ancho_camion),distancia_parte_e)
                except Exception as e:
                    self.escribirArchivoLog("Error Calculo MR estéreo: "+str(e))
                #============   FIN Calculo de Metro Ruma Estereo   ===========

                #============   Calculo de Metro Ruma Largo Calculado   ===========
                #=============   Obtener Largo de trozo Automatico en función a la velocidad del camión   ============
                try:
                    distancia_parte_lc = self.obtener_distancia_parte(indice_inicio_parte,
                                                                indice_fin_parte,
                                                                ruta_camion,
                                                                velocidad_camion)
                except Exception as e:
                    self.escribirArchivoLog("Error obtener_distancia_parte(): "+str(e))
                
                try:
                    mtr_ruma_lc = self.calculo_metro_ruma_estereo(prom_h_full_lc,float(CONSTANTE_ANCHO),distancia_parte_lc)
                except Exception as e:
                    self.escribirArchivoLog("Error Calculo MR LC: "+str(e))
                #============   FIN Calculo de Metro Ruma Largo Calculado   ===========

                #============   Guardar variables globales   ===========   
                #Datos Medición estereo con largo suministrado
                self.ALTURA_CAMION_E = prom_h_full
                self.LARGO_CAMION_E = distancia_parte_e
                self.BASE_1_CAMION_E = base_grupo1
                self.BASE_2_CAMION_E = base_grupo2
                self.ANCHO_CAMION_E = ancho_camion
                self.MR_CAMION_E = mtr_ruma_e

                #Datos Medición con largo calculado
                self.VELOCIDAD_PROMEDIO_TOTAL = velocidad_camion
                self.ALTURA_CAMION_LC = prom_h_full_lc
                self.BASE_1_CAMION_LC = CONSTANTE_BASE
                self.BASE_2_CAMION_LC = CONSTANTE_BASE
                self.ANCHO_CAMION_LC = CONSTANTE_ANCHO
                self.LARGO_CAMION_LC = distancia_parte_lc
                self.MR_CAMION_LC = mtr_ruma_lc
                
                # print("**** DATOS CAMION ****")
                # print("Promedio de Altura: ",prom_h_full)
                # print("Base 1: ",base_grupo1)
                # print("Base 2: ",base_grupo2)
                # print("Ancho Camión: ",ancho_camion)
                # print("MR E: ",mtr_ruma_e)
                # print("Promedio de Altura LC: ",prom_h_full_lc)
                # print("Base CONSTANTE: ",CONSTANTE_BASE)
                # print("Ancho Camión CONSTANTE: ",CONSTANTE_ANCHO)
                # print("Largo Trozo Calculado: ",distancia_parte_lc)
                # print("MR LC: ",mtr_ruma_lc)
                #============   FIN Guardar variables globales   ===========
            else:

                #=============   Obtener Largo de trozo Automatico en función a la velocidad del camión   ============
                try:
                    distancia_parte = self.obtener_distancia_parte(indice_inicio_parte,
                                                                indice_fin_parte,
                                                                ruta_camion,
                                                                velocidad_camion)
                except Exception as e:
                    self.escribirArchivoLog("Error obtener_distancia_parte(): "+str(e))
                
                try:
                    mtr_ruma_lc = self.calculo_metro_ruma_estereo(prom_h_full_lc,float(CONSTANTE_ANCHO),distancia_parte_lc)
                except Exception as e:
                    self.escribirArchivoLog("Error Calculo MR LC: "+str(e))
                #=============   FIN Obtener Largo de trozo Automatico en función a la velocidad del camión   ===========

                #============   Guardar variables globales   ===========   
                #Datos Medición con largo calculado
                self.VELOCIDAD_PROMEDIO_TOTAL = velocidad_camion
                self.ALTURA_CAMION_LC = prom_h_full_lc
                self.BASE_1_CAMION_LC = CONSTANTE_BASE
                self.BASE_2_CAMION_LC = CONSTANTE_BASE
                self.ANCHO_CAMION_LC = CONSTANTE_ANCHO
                self.LARGO_CAMION_LC = distancia_parte_lc
                self.MR_CAMION_LC = mtr_ruma_lc

                # print("**** DATOS CAMION ****")
                # print("Promedio de Altura LC: ",prom_h_full_lc)
                # print("Base CONSTANTE: ",CONSTANTE_BASE)
                # print("Ancho Camión CONSTANTE: ",CONSTANTE_ANCHO)
                # print("Largo Trozo Calculado: ",distancia_parte_lc)
                # print("MR LC: ",mtr_ruma_lc)
                #============   FIN Guardar variables globales   ===========
        
        
        return lista_graficar
        
    def obtener_velocidad_camion(self,inicio_camion : int,
                                fin_carro : int,
                                ruta_camion : str):
        #print(ruta_camion)
        lineas = self.get_lines_in_file(ruta_camion+'/'+self.PES_ID+'.txt')
        tiempo_final = 0
        # print(lineas)

        for x, linea in enumerate(lineas):
            
            if x == inicio_camion:
                datos_por_linea = linea.split(';')
                tiempo_inicio = datos_por_linea[2]
            if x == fin_carro:
                datos_por_linea = linea.split(';')
                tiempo_final = datos_por_linea[2]
        
        tiempo_camion = int(tiempo_final) - int(tiempo_inicio)
        tiempo_camion = tiempo_camion / 3600

        velocidad_camion = round(0.019 / tiempo_camion,2)
        # print("Velocidad total",str(velocidad_camion)+'Km/H')
        # print("Tiempor Camión",tiempo_camion)
        self.VELOCIDAD_PROMEDIO_TOTAL = velocidad_camion

        return velocidad_camion

    def obtener_distancia_parte(self,indice_inicio_parte : int,
                                        indice_fin_parte: int,
                                        ruta_camion : str,
                                        velocidad_camion: float):
        lineas = self.get_lines_in_file(ruta_camion+'/'+self.PES_ID+'.txt')
        tiempo_final = 0

        for x, linea in enumerate(lineas):
            
            if x == indice_inicio_parte:
                datos_por_linea = linea.split(';')
                tiempo_inicio = datos_por_linea[2]
            if x == indice_fin_parte:
                datos_por_linea = linea.split(';')
                tiempo_final = datos_por_linea[2]
        
        tiempo_parte = int(tiempo_final) - int(tiempo_inicio)
        tiempo_parte = tiempo_parte / 3600

        distancia_parte = (velocidad_camion * tiempo_parte) * 1000

        if distancia_parte > 7.32:
            distancia_parte = 7.32

        if distancia_parte < 2.4:
            distancia_parte = 2.4


        #print('Distancia',str(distancia_parte)+'M')
        return distancia_parte

    def calculo_metro_ruma_estereo(self, altura : float, ancho : float, largo : float):
        #print(altura,ancho,largo)
        #Limites de cada variable
        if altura < 1:
            altura = 1
        if altura > 3:
            altura = 3

        if ancho < 2.3:
            ancho = 2.3
        if ancho > 3:
            ancho = 3

        if largo < 2.4:
            largo = 2.4
        if largo > 7.32:
            largo = 7.32

        metro_cubico_estereo = altura * ancho * largo
        metro_ruma = metro_cubico_estereo / 2.44
        if metro_ruma > 20:
            metro_ruma = 20
        return round(metro_ruma,2)
    
    def post_graficar_3D(self,lista_graficar_camion : list, 
                                lista_graficar_carro : list):
        try:
            id_camion = self.PES_ID
            #Instanciar Modulos TTC
            #ttc = TTC()
            #Obtener ruta directorio actual
            #ruta_directorio = ttc.obtener_ruta_actual()
            #Ruta archivos configuraciones.json
            ruta_configuraciones = "E:/TTC/TTC_SCAN/SETUP/configuraciones.json"
            #Ruta todos los camiones
            ruta_camiones = self.cargar_datos(ruta_configuraciones,"Ruta-Camiones")
            limites_graficar_3d = self.cargar_datos(ruta_configuraciones,"LIMITES-GRAFICAR-3D")
            #Ruta camión específico para medir
            ruta_camion = ruta_camiones+"/"+id_camion
            lista_original_Top_x = self.cargar_datos(ruta_camion+'/Puntos-Top.json','x')
            lista_original_Top_y = self.cargar_datos(ruta_camion+'/Puntos-Top.json','y')
            lista_original_Top_z = self.cargar_datos(ruta_camion+'/Puntos-Top.json','z')
            lista_original_Top_x = self.regenerete_list_z(lista_original_Top_x)
            lista_original_Top_y = self.regenerete_list_z(lista_original_Top_y)
            lista_original_Top_z = self.regenerete_list_z(lista_original_Top_z)
            lista_top_x = []
            lista_top_y = []
            lista_top_z = []

            for i, top in enumerate(lista_original_Top_z):
                lista_top_z.append(top[limites_graficar_3d[0]:limites_graficar_3d[1]])
                lista_top_x.append(lista_original_Top_x[i][limites_graficar_3d[0]:limites_graficar_3d[1]])
                lista_top_y.append(lista_original_Top_y[i][limites_graficar_3d[0]:limites_graficar_3d[1]])

            
            
            inicio_cabina =  int(self.INICIOCABINA_AP)
            inicio_camion =  int(self.INICIOCAMION_AP)
            fin_camion = int(self.FINCAMION_AP)
            inicio_carro = int(self.INICIOCARRO_AP)
            fin_carro = int(self.FINCARRO_AP)
            
            """        
            print("inicio_cabina:",inicio_cabina)
            print("inicio_camion:",inicio_camion)
            print("fin_camion:",fin_camion)
            print("inicio_carro:",inicio_carro)
            print("fin_carro:",fin_carro)
            """

            graficar = {}
            dot_camion = 1
            dot_partes = 5
            
            
            #Mostrar gráfica 2D estereo
            if inicio_camion != 0 and \
                fin_camion != 0 and \
                inicio_carro != 0 and \
                fin_carro != 0:

                #Obtener Inicio y Fin desde los labels
                print("MR TOTAL E",self.MR_TOTAL_E)
                MR_TOTALCAMION = 'MR_TOTAL='+str(round((self.MR_CAMION_E + self.MR_CARRO_E),2))

                print("MR CAMION E",self.MR_CAMION_E)
                MR_CAMION = 'MR_CAMION='+str(self.MR_CAMION_E)

                print("MR CARRO E",self.MR_CARRO_E)
                MR_CARRO = 'MR_CARRO='+str(self.MR_CARRO_E)

                #Separación Partes
                camion_z = lista_top_z[inicio_cabina:fin_carro]
                camion_y = lista_top_y[inicio_cabina:fin_carro]
                camion_x = lista_top_x[inicio_cabina:fin_carro]
                # print(camion_z[-1])
                # print(len(camion_z[0]))

                x_carro, y_carro, z_carro = self.listas_para_graficar_3D(camion_x,camion_z,camion_y)
                x_b4, y_b4, z_b4 = self.listas_para_graficar_3D_estereo(lista_graficar_camion)
                x_b5, y_b5, z_b5 = self.listas_para_graficar_3D_estereo(lista_graficar_carro)
                graficar['CamionTotal'] = [x_carro, y_carro, z_carro, dot_camion,MR_TOTALCAMION]
                graficar['Camion'] = [x_b4, y_b4, z_b4,dot_partes,MR_CAMION]
                graficar['Carro'] = [x_b5, y_b5, z_b5,dot_partes,MR_CARRO]

            #self.graficar3d_nveces(carpeta = id_camion,kwargs = graficar) 
            self.graficar3d_nveces(carpeta = id_camion,ruta_camion = ruta_camion,kwargs = graficar) 
        except Exception as e:
            self.escribirArchivoLog("Error al graficar 3D: "+str(e))
      
    def post_graficar_3D_lateral(self,lista_graficar_camion : list, 
                                lista_graficar_carro : list,
                                lado_camion : str):
        try:
            id_camion = self.PES_ID
            #Instanciar Modulos TTC
            #ttc = TTC()
            #Obtener ruta directorio actual
            #ruta_directorio = ttc.obtener_ruta_actual()
            #Ruta archivos configuraciones.json
            ruta_configuraciones = "E:/TTC/TTC_SCAN/SETUP/configuraciones.json"
            #Ruta todos los camiones
            ruta_camiones = self.cargar_datos(ruta_configuraciones,"Ruta-Camiones")
            limites_graficar_3d = [5,20] #self.cargar_datos(ruta_configuraciones,"LIMITES-GRAFICAR-3D")
            #Ruta camión específico para medir
            ruta_camion = ruta_camiones+"/"+id_camion

            if lado_camion == 'bottom':
                lista_original_Bottom_x = self.cargar_datos(ruta_camion+'/Puntos-Bottom.json','x')
                lista_original_Bottom_y = self.cargar_datos(ruta_camion+'/Puntos-Bottom.json','y')
                lista_original_Bottom_z = self.cargar_datos(ruta_camion+'/Puntos-Bottom.json','z')            
            elif lado_camion == 'right':
                lista_original_Bottom_x = self.cargar_datos(ruta_camion+'/Puntos-Right.json','x')
                lista_original_Bottom_y = self.cargar_datos(ruta_camion+'/Puntos-Right.json','y')
                lista_original_Bottom_z = self.cargar_datos(ruta_camion+'/Puntos-Right.json','z')  

            lista_original_Bottom_x = self.regenerete_list_z(lista_original_Bottom_x)
            lista_original_Bottom_y = self.regenerete_list_z(lista_original_Bottom_y)
            lista_original_Bottom_z = self.regenerete_list_z(lista_original_Bottom_z)

            lista_Bottom_x = []
            lista_Bottom_y = []
            lista_Bottom_z = []

            for i, Bottom in enumerate(lista_original_Bottom_z):
                lista_Bottom_z.append(Bottom)
                lista_Bottom_x.append(lista_original_Bottom_x[i])
                lista_Bottom_y.append(lista_original_Bottom_y[i])

            
            
            inicio_cabina =  int(self.INICIOCABINA_AP)
            inicio_camion =  int(self.INICIOCAMION_AP)
            fin_camion = int(self.FINCAMION_AP)
            inicio_carro = int(self.INICIOCARRO_AP)
            fin_carro = int(self.FINCARRO_AP)
            
            """        
            print("inicio_cabina:",inicio_cabina)
            print("inicio_camion:",inicio_camion)
            print("fin_camion:",fin_camion)
            print("inicio_carro:",inicio_carro)
            print("fin_carro:",fin_carro)
            """

            graficar = {}
            dot_camion = 1
            dot_partes = 5
            
            
            #Mostrar gráfica 2D estereo
            if inicio_camion != 0 and \
                fin_camion != 0 and \
                inicio_carro != 0 and \
                fin_carro != 0:

                #Obtener Inicio y Fin desde los labels
                if lado_camion == 'bottom':

                    # print("Lista Graficar camion Bottom:",lista_graficar_camion)
                    # print("Lista Graficar carro Bottom:",lista_graficar_carro)

                    cont_camion=0
                    graficar_camion = False
                    cont_carro=0
                    graficar_carro = False

                    for val in lista_graficar_camion:
                        if val[1] == []:
                            cont_camion+=1
                    
                    if cont_camion < 1:
                        graficar_camion = False 
                    else:
                        graficar_camion = True 

                    for val in lista_graficar_carro:
                        if val[1] == []:
                            cont_carro+=1
                    
                    if cont_carro < 1:
                        graficar_carro = False   
                    else:
                        graficar_carro = True   

                    MR_TOTALCAMION = 'MR_TOTAL='+str(round((self.MR_CAMION_E + self.MR_CARRO_E),2))

                    
                    DST_CAMION = 'DST_CAMION='+str(self.DISTANCIA_CAMION_BOTTOM)

                    
                    DST_CARRO = 'DST_CARRO='+str(self.DISTANCIA_CARRO_BOTTOM)                   

                    #Separación Partes
                    camion_z = lista_Bottom_z[inicio_cabina:fin_carro]
                    camion_y = lista_Bottom_y[inicio_cabina:fin_carro]
                    camion_x = lista_Bottom_x[inicio_cabina:fin_carro]
                    # print(camion_z[-1])
                    # print(len(camion_z[0]))


                    if graficar_camion == True:
                        x_b4, y_b4, z_b4 = self.listas_para_graficar_3D_estereo_lateral(lista_graficar_camion,lado_camion)
                        graficar['Camion'] = [x_b4, y_b4, z_b4,dot_partes,DST_CAMION]

                    if graficar_carro == True:
                        x_b5, y_b5, z_b5 = self.listas_para_graficar_3D_estereo_lateral(lista_graficar_carro,lado_camion)
                        graficar['Carro'] = [x_b5, y_b5, z_b5,dot_partes,DST_CARRO]

                    x_carro, y_carro, z_carro = self.listas_para_graficar_3D_lateral(camion_x,camion_z,camion_y)
                    graficar['CamionTotal'] = [x_carro, y_carro, z_carro, dot_camion,MR_TOTALCAMION]



                elif lado_camion == 'right':

                    # print("Lista Graficar camion Right:",lista_graficar_camion)
                    # print("Lista Graficar carro Right:",lista_graficar_carro)

                    cont_camion=0
                    graficar_camion = False
                    cont_carro=0
                    graficar_carro = False

                    for val in lista_graficar_camion:
                        if val[1] == []:
                            cont_camion+=1
                    
                    if cont_camion < 1:
                        graficar_camion = False 
                    else:
                        graficar_camion = True 

                    for val in lista_graficar_carro:
                        if val[1] == []:
                            cont_carro+=1
                    
                    if cont_carro < 1:
                        graficar_carro = False  
                    else:
                        graficar_carro = True 

                    MR_TOTALCAMION = 'MR_TOTAL='+str(round((self.MR_CAMION_E + self.MR_CARRO_E),2))

                    
                    DST_CAMION = 'DST_CAMION='+str(self.DISTANCIA_CAMION_RIGHT)

                    
                    DST_CARRO = 'DST_CARRO='+str(self.DISTANCIA_CARRO_RIGHT)

                    #Separación Partes
                    camion_z = lista_Bottom_z[inicio_cabina:fin_carro]
                    camion_y = lista_Bottom_y[inicio_cabina:fin_carro]
                    camion_x = lista_Bottom_x[inicio_cabina:fin_carro]
                    # print(camion_z[-1])
                    # print(len(camion_z[0]))


                    if graficar_camion == True:
                        x_b4, y_b4, z_b4 = self.listas_para_graficar_3D_estereo_lateral(lista_graficar_camion,
                                                                                    lado_camion)
                        graficar['Camion'] = [x_b4, y_b4, z_b4,dot_partes,DST_CAMION]

                    if graficar_carro == True:
                        x_b5, y_b5, z_b5 = self.listas_para_graficar_3D_estereo_lateral(lista_graficar_carro,
                                                                                    lado_camion)
                        graficar['Carro'] = [x_b5, y_b5, z_b5,dot_partes,DST_CARRO]
                    
                    x_carro, y_carro, z_carro = self.listas_para_graficar_3D_lateral(camion_x,camion_z,camion_y)
                    graficar['CamionTotal'] = [x_carro, y_carro, z_carro, dot_camion,MR_TOTALCAMION]

            #self.graficar3d_nveces(carpeta = id_camion,kwargs = graficar) 
            self.graficar3d_nveces_laterales(carpeta = id_camion,
                                                ruta_camion = ruta_camion,
                                                nombre_archivo = lado_camion,
                                                kwargs = graficar) 
        except Exception as e:
            self.escribirArchivoLog("Error al graficar 3D: "+str(e))
     
    def post_graficar_3D_autocargante(self,lista_graficar_autocargante : list, 
                                            lista_graficar_carro : list):
        try:
            id_camion = self.PES_ID
            #Instanciar Modulos TTC
            #ttc = TTC()
            #Obtener ruta directorio actual
            #ruta_directorio = ttc.obtener_ruta_actual()
            #Ruta archivos configuraciones.json
            ruta_configuraciones = "E:/TTC/TTC_SCAN/SETUP/configuraciones.json"
            #Ruta todos los camiones
            ruta_camiones = self.cargar_datos(ruta_configuraciones,"Ruta-Camiones")
            limites_graficar_3d = self.cargar_datos(ruta_configuraciones,"LIMITES-GRAFICAR-3D")
            #Ruta camión específico para medir
            ruta_camion = ruta_camiones+"/"+id_camion
            lista_original_Top_x = self.cargar_datos(ruta_camion+'/Puntos-Top.json','x')
            lista_original_Top_y = self.cargar_datos(ruta_camion+'/Puntos-Top.json','y')
            lista_original_Top_z = self.cargar_datos(ruta_camion+'/Puntos-Top.json','z')
            lista_original_Top_x = self.regenerete_list_z(lista_original_Top_x)
            lista_original_Top_y = self.regenerete_list_z(lista_original_Top_y)
            lista_original_Top_z = self.regenerete_list_z(lista_original_Top_z)
            lista_top_x = []
            lista_top_y = []
            lista_top_z = []

            for i, top in enumerate(lista_original_Top_z):
                lista_top_z.append(top[limites_graficar_3d[0]:limites_graficar_3d[1]])
                lista_top_x.append(lista_original_Top_x[i][limites_graficar_3d[0]:limites_graficar_3d[1]])
                lista_top_y.append(lista_original_Top_y[i][limites_graficar_3d[0]:limites_graficar_3d[1]])
            
            inicio_cabina =  int(self.INICIOCABINA_AP)
            inicio_camion =  int(self.INICIOCAMION_AP)
            fin_camion = int(self.FINCAMION_AP)
            inicio_carro = int(self.INICIOCARRO_AP)
            fin_carro = int(self.FINCARRO_AP)
            
            """        
            print("inicio_cabina:",inicio_cabina)
            print("inicio_camion:",inicio_camion)
            print("fin_camion:",fin_camion)
            print("inicio_carro:",inicio_carro)
            print("fin_carro:",fin_carro)
            """

            graficar = {}
            dot_camion = 1
            dot_partes = 5
            
            
            #Mostrar gráfica 2D estereo
            if inicio_camion != 0 and \
                fin_camion != 0 and \
                inicio_carro != 0 and \
                fin_carro != 0:

                #Obtener Inicio y Fin desde los labels
                MR_TOTALCAMION = 'MR_TOTAL='+str(self.MR_TOTAL_E)

                MR_CAMION = 'MR_CAMION='+str(self.MR_CAMION_E)

                MR_CARRO = 'MR_CARRO='+str(self.MR_CARRO_E)

                #Separación Partes
                camion_z = lista_top_z[inicio_cabina:fin_carro]
                camion_y = lista_top_y[inicio_cabina:fin_carro]
                camion_x = lista_top_x[inicio_cabina:fin_carro]

                x_carro, y_carro, z_carro = self.listas_para_graficar_3D(camion_x,camion_z,camion_y)
                x_b4, y_b4, z_b4 = self.listas_para_graficar_3D_estereo(lista_graficar_autocargante)
                x_b5, y_b5, z_b5 = self.listas_para_graficar_3D_estereo(lista_graficar_carro)
                graficar['CamionTotal'] = [x_carro, y_carro, z_carro, dot_camion,MR_TOTALCAMION]
                graficar['Camion'] = [x_b4, y_b4, z_b4,dot_partes,MR_CAMION]
                graficar['Carro'] = [x_b5, y_b5, z_b5,dot_partes,MR_CARRO]

            #self.graficar3d_nveces(carpeta = id_camion,kwargs = graficar) 
            self.graficar3d_nveces(carpeta = id_camion,ruta_camion = ruta_camion,kwargs = graficar) 
        except Exception as e:
            self.escribirArchivoLog("Error al graficar 3D: "+str(e))
  
    def graficar3d_nveces(self, carpeta : str, ruta_camion : str, kwargs):
        try:
            if not exists(ruta_camion+"/Img/Graficas"):
                makedirs(ruta_camion+"/Img/Graficas")
            else:
                pass
                #print('Ya existe la carpeta se va a borrar')
                # shutil.rmtree(ruta_camion+"/Img/Graficas", ignore_errors=True)
                # time.sleep(2)
                # makedirs(ruta_camion+"/Img/Graficas")

            #Iniciar Variables en 0
            MR_CAMION = 0
            MR_CARRO = 0
            MR_TOTAL = 0
            #Estructura matplotlib eje cartesiano 3D
            #Quitar Menu Superior
            rcParams['toolbar'] = 'None'
            fig, axs = plt.subplots(figsize=(15,8),frameon=False)
            #Quitar el margen de los bordes
            plt.subplots_adjust(left=0, right=1, top=1, bottom=0)

            #fig_ = plt.get_current_fig_manager()
            #fig_.window.showMaximized()
            #fig_.window.setWindowIcon(QtGui.QIcon("img/Icono-APP.jpg"))
            #fig_.canvas.set_window_title('TTC Gráfica 3D')
            
            axs = plt.axes(projection='3d')
            for key in kwargs:
                axs.scatter(np.array(kwargs[key][0]), 
                                        np.array(kwargs[key][2]), 
                                        np.array(kwargs[key][1]),
                                        marker='o',
                                        s=kwargs[key][3]**2,
                                        label="TTC-SCAN {} {}".format(carpeta,key))
                """if 'AUTOCARGANTE' in key:
                    axs.text(kwargs[key][0][0],kwargs[key][2][4],380, key,'x', fontsize=12,color = 'black')
                if 'MR=' in kwargs[key][4]:
                    axs.text(kwargs[key][0][0],kwargs[key][2][4],400, kwargs[key][4],'x', fontsize=9,color = 'black')
                """
                if 'MR_CAMION' in kwargs[key][4]:
                    MR_CAMION = kwargs[key][4]
                    MR_CAMION = MR_CAMION.split('=')[1]
                    centro_camion = kwargs[key][0][0] +( (kwargs[key][0][-1] - kwargs[key][0][0]) / 4 )
                    axs.text(centro_camion,kwargs[key][2][4],480, kwargs[key][4],'x', fontsize=15,color = 'black')
                if 'MR_CARRO' in kwargs[key][4]:
                    MR_CARRO = kwargs[key][4]
                    MR_CARRO = MR_CARRO.split('=')[1]
                    centro_carro = kwargs[key][0][0] +( (kwargs[key][0][-1] - kwargs[key][0][0]) / 4 )
                    axs.text(centro_carro,kwargs[key][2][4],480, kwargs[key][4],'x', fontsize=15,color = 'black')
                """if 'MR_TOTALCAMION' in kwargs[key][2]:
                    MR_TOTAL = kwargs[key][2]
                    MR_TOTALCAMION = MR_TOTALCAMION.split('=')[1]
                    left, right = plt.xlim()
                    mid = left + ( (right - left) / 3 )
                    axs.text(mid,500,550, kwargs[key][2],'x', fontsize=18,color = 'black')
                """

            #Agregar la leyenda
            #axs.legend()
            #Configuración para la rotación de la primera visualización
            axs.view_init(13, -91)
            #Configuración del eje Y para visualizar mejor la nube de puntos
            axs.set(ylim=(-800, 1500))
            axs.set_zlim3d(200,600)
            #Quitar los ejes
            plt.axis('off')
            
            
            # MR_TOTAL = round(float(MR_CAMION) + float(MR_CARRO),2)
            # print(self.MR_TOTAL_E)
            MR_TOTAL = round((self.MR_CAMION_E + self.MR_CARRO_E),2)
            # print(MR_TOTAL)

            left, right = plt.xlim()
            mid = left + ( (right - left) / 3 )
            axs.text(mid,500,550, 'TOTAL_MR= '+str(MR_TOTAL),'x', fontsize=18,color = 'black')

            #Imprimir Gráfica
            #figure = plt.gcf()
            #figure.set_size_inches(5, 1)
            #plt.tight_layout()
            plt.savefig(ruta_camion+"/Img/Graficas/Grafica3D.jpg")
            # print("Graficas camión ID:",)
            plt.savefig("E:/TTC/GRAFICAS/Grafica3D-"+str(self.PES_ID)+".jpg")
            # plt.show()
            plt.close()
        except Exception as e:
            self.escribirArchivoLog("Error Graficar 3D N_Veces: "+str(e))
  
    def graficar3d_nveces_laterales(self, carpeta : str, 
                                            ruta_camion : str, 
                                            nombre_archivo : str,
                                            kwargs):
        try:
            if not exists(ruta_camion+"/Img/Graficas"):
                makedirs(ruta_camion+"/Img/Graficas")
            else:
                pass
                #print('Ya existe la carpeta se va a borrar')
                # shutil.rmtree(ruta_camion+"/Img/Graficas", ignore_errors=True)
                # time.sleep(2)
                # makedirs(ruta_camion+"/Img/Graficas")

            #Iniciar Variables en 0
            DST_CAMION = 0
            DST_CARRO = 0
            # MR_TOTAL = 0
            #Estructura matplotlib eje cartesiano 3D
            #Quitar Menu Superior
            rcParams['toolbar'] = 'None'
            fig, axs = plt.subplots(figsize=(15,8),frameon=False)
            #Quitar el margen de los bordes
            plt.subplots_adjust(left=0, right=1, top=1, bottom=0)

            # fig_ = plt.get_current_fig_manager()
            # fig_.window.showMaximized()
            # fig_.window.setWindowIcon(QtGui.QIcon("Icono-APP.ico"))
            # fig_.canvas.set_window_title('TTC Gráfica 3D')
            
            axs = plt.axes(projection='3d')
            for key in kwargs:
                axs.scatter(np.array(kwargs[key][0]), 
                                        np.array(kwargs[key][2]), 
                                        np.array(kwargs[key][1]),
                                        marker='o',
                                        s=kwargs[key][3]**2,
                                        label="TTC-SCAN {} {}".format(carpeta,key))
                """if 'AUTOCARGANTE' in key:
                    axs.text(kwargs[key][0][0],kwargs[key][2][4],380, key,'x', fontsize=12,color = 'black')
                if 'MR=' in kwargs[key][4]:
                    axs.text(kwargs[key][0][0],kwargs[key][2][4],400, kwargs[key][4],'x', fontsize=9,color = 'black')
                # """
                if 'DST_CAMION' in kwargs[key][4]:
                    DST_CAMION = kwargs[key][4]
                    DST_CAMION = DST_CAMION.split('=')[1]
                    centro_camion = kwargs[key][0][0] +( (kwargs[key][0][-1] - kwargs[key][0][0]) / 4 )
                    axs.text(centro_camion,kwargs[key][2][4],500, kwargs[key][4],'x', fontsize=15,color = 'black')
                    axs.text(centro_camion,kwargs[key][2][4],600, 'ANCHO CAMION = '+str(self.ANCHO_CAMION_CALCULADO),
                                'x', fontsize=15,color = 'black')
                if 'DST_CARRO' in kwargs[key][4]:
                    DST_CARRO = kwargs[key][4]
                    DST_CARRO = DST_CARRO.split('=')[1]
                    centro_carro = kwargs[key][0][0] +( (kwargs[key][0][-1] - kwargs[key][0][0]) / 4 )
                    axs.text(centro_carro,kwargs[key][2][4],500, kwargs[key][4],'x', fontsize=15,color = 'black')
                    axs.text(centro_carro,kwargs[key][2][4],600, 'ANCHO CARRO = '+str(self.ANCHO_CARRO_CALCULADO),
                                'x', fontsize=15,color = 'black')
                """if 'MR_TOTALCAMION' in kwargs[key][2]:
                    MR_TOTAL = kwargs[key][2]
                    MR_TOTALCAMION = MR_TOTALCAMION.split('=')[1]
                    left, right = plt.xlim()
                    mid = left + ( (right - left) / 3 )
                    axs.text(mid,500,550, kwargs[key][2],'x', fontsize=18,color = 'black')
                """

            #Agregar la leyenda
            #axs.legend()
            #Configuración para la rotación de la primera visualización
            # axs.view_init(13, -91)
            axs.view_init(-44, -90)
            #Configuración del eje Y para visualizar mejor la nube de puntos
            axs.set(ylim=(-800, 1500))
            axs.set_zlim3d(200,600)
            #Quitar los ejes
            plt.axis('off')
            
            
            # MR_TOTAL = round(float(MR_CAMION) + float(MR_CARRO),2)
            # # print(self.MR_TOTAL_E)
            # MR_TOTAL = round((self.MR_CAMION_E + self.MR_CARRO_E),2)
            # # print(MR_TOTAL)

            left, right = plt.xlim()
            mid = left + ( (right - left) / 3 )
            axs.text(mid,500,750, 'DISTANCIA LADO '+str(nombre_archivo)+' ID = '+str(self.PES_ID),'x', fontsize=18,color = 'black')

            #Imprimir Gráfica
            #figure = plt.gcf()
            #figure.set_size_inches(5, 1)
            #plt.tight_layout()
            plt.savefig(ruta_camion+"/Img/Graficas/Grafica3D-"+str(nombre_archivo)+"-"+str(self.PES_ID)+".jpg")
            # print("Graficas camión ID:",)
            # plt.savefig("E:/TTC/GRAFICAS/Grafica3D-"+str(nombre_archivo)+"-"+str(self.PES_ID)+".jpg")
            # plt.show()
            plt.close()
        except Exception as e:
            self.escribirArchivoLog("Error Graficar 3D N_Veces Laterales"+str(nombre_archivo)+": "+str(e))

    def listas_para_graficar_3D(self,lista_camion_carro_x : list,
                                        lista_camion_carro_z : list,
                                        lista_camion_carro_y : list) -> list:
        try:            
            # ruta_directorio = ttc.obtener_ruta_actual()
            #Ruta archivos configuraciones.json
            ruta_configuraciones = "E:/TTC/TTC_SCAN/SETUP/configuraciones.json"
            #Ruta todos los camiones
            altura_portal = self.cargar_datos(ruta_configuraciones,"ALTURA-PORTAL")


            np_lista_z = np.array(lista_camion_carro_z)
            np_lista_x = np.array(lista_camion_carro_x)
            np_lista_y = np.array(lista_camion_carro_y)

            # print(np_lista_y)

            #Inicializando Nuevos vectores
            xi = []
            yi = []
            zi = []
            #Loop para filtrar
            for idx, valx in enumerate(np_lista_z):
                for i, val in enumerate(valx):
                    if int(val) >= 60 and int(val) <= 350:
                        yi.append(altura_portal-int(val))
                        xi.append(np_lista_x[idx][i])
                        zi.append(((int(i)+7)*20))

            return xi, yi, zi
        except Exception as e:
            self.escribirArchivoLog("Error listas_para_graficar_3D: "+str(e))

    def listas_para_graficar_3D_lateral(self,lista_camion_carro_x : list,
                                        lista_camion_carro_z : list,
                                        lista_camion_carro_y : list) -> list:
        try:            
            # ruta_directorio = ttc.obtener_ruta_actual()
            #Ruta archivos configuraciones.json
            # ruta_configuraciones = "E:/TTC/TTC_SCAN/SETUP/configuraciones.json"
            # #Ruta todos los camiones
            # altura_portal = self.cargar_datos(ruta_configuraciones,"ALTURA-PORTAL")


            np_lista_z = np.array(lista_camion_carro_z)
            np_lista_x = np.array(lista_camion_carro_x)
            np_lista_y = np.array(lista_camion_carro_y)

            # print(np_lista_y)

            #Inicializando Nuevos vectores
            xi = []
            yi = []
            zi = []
            #Loop para filtrar
            for idx, valx in enumerate(np_lista_z):
                for i, val in enumerate(valx):
                    if int(val) >= 50 and int(val) <= 250:
                        yi.append(int(val))
                        xi.append(np_lista_x[idx][i])
                        zi.append(((int(i)+2)*20))

            return xi, yi, zi
        except Exception as e:
            self.escribirArchivoLog("Error listas_para_graficar_3D lateral: "+str(e))

    def listas_para_graficar_3D_estereo(self,lista_graficar : list) -> list:
        try:
            
            ruta_configuraciones = "E:/TTC/TTC_SCAN/SETUP/configuraciones.json"
            #Ruta todos los camiones
            altura_portal = self.cargar_datos(ruta_configuraciones,"ALTURA-PORTAL")
            #print(np_lista_z,np_lista_x,np_lista_y)
            # print(lista_graficar[-1])

            #Inicializando Nuevos vectores
            xi = []
            yi = []
            zi = []
            #Loop para filtrar
            for valx in enumerate(lista_graficar):
                for x in valx[1][1]:
                    # print(x)
                    yi.append(altura_portal-int(x[1]))
                    xi.append(valx[1][0])
                    zi.append(((int(x[0])+7)*20))

            return xi, yi, zi
        except Exception as e:
            self.escribirArchivoLog("Error listas_para_graficar_3D estereo: "+str(e))

    def listas_para_graficar_3D_estereo_lateral(self,lista_graficar : list, nombre_lado : str) -> list:
        try:
            # ruta_configuraciones = "E:/TTC/TTC_SCAN/SETUP/configuraciones.json"
            #Ruta todos los camiones
            # altura_portal = self.cargar_datos(ruta_configuraciones,"ALTURA-PORTAL")
            #print(np_lista_z,np_lista_x,np_lista_y)
            
            # print(self.lista_graficar_camion_bottom,self.lista_graficar_camion_bottom)
            # print("lista_graficar",lista_graficar)
            # print("Len lista_graficar",len(lista_graficar))
            # if nombre_lado == 'right':
                # print(nombre_lado)
                # print("lista_graficar",nombre_lado,lista_graficar)

            #Inicializando Nuevos vectores
            xi = []
            yi = []
            zi = []
            #Loop para filtrar
            for i, valx in enumerate(lista_graficar):
                # print(i,"-- valx --",valx)
                for x in valx[1]:
                    # print(x)
                    yi.append(int(x[1]))
                    xi.append(valx[0])
                    zi.append(((int(x[0])+2)*20))

            return xi, yi, zi
        except Exception as e:
            self.escribirArchivoLog("Error listas_para_graficar_3D estereo lateral: "+str(e))

    def listas_para_graficar_3D_autocargante(self,lista_graficar : list) -> list:
        try:
            
            ruta_configuraciones = "E:/TTC/TTC_SCAN/SETUP/configuraciones.json"
            #Ruta todos los camiones
            altura_portal = self.cargar_datos(ruta_configuraciones,"ALTURA-PORTAL")
            #print(np_lista_z,np_lista_x,np_lista_y)

            #Inicializando Nuevos vectores
            xi = []
            yi = []
            zi = []
            #Loop para filtrar
            for valx in enumerate(lista_graficar):
                for x in valx[1][1]:
                    #print(x)
                    yi.append(altura_portal-int(x[1]))
                    xi.append(valx[1][0])
                    zi.append(((int(x[0])+7)*20))

            return xi, yi, zi
        except Exception as e:
            self.escribirArchivoLog("Error listas_para_graficar_3D: "+str(e))

    def post_estado_txt(self,path : str, nombre : str, mensaje : str):
        try:
            ruta_directorio = self.obtener_ruta_actual()
            ruta_configuraciones = ruta_directorio+"/SETUP/configuraciones.json"
            ruta_exportar_camiones = self.cargar_datos(ruta_configuraciones, 'Path-Exportar')

            ruta_destino = ruta_exportar_camiones+"/"+str(self.PES_ID)

            print("Escribiendo FINALIZADO")
            f = open(ruta_destino+'/'+nombre+'.txt', "w")
            f.write(mensaje)
            f.close()
        except Exception as e:
            self.escribirArchivoLog("Error Escribir Archivo estado.txt: "+str(e))
  
    def post_info_camaras_txt(self,path : str, nombre : str, mensaje : str):
        try:
            f = open(path+'/'+nombre+'.txt', "w")
            f.write(mensaje)
            f.close()
        except Exception as e:
            self.escribirArchivoLog("Error Escribir Archivo INFO-CAMARAS.txt: "+str(e))
    
    def post_archivo_txt(self,mensaje : str):
        try:
            ruta_directorio = self.obtener_ruta_actual()
            ruta_configuraciones = ruta_directorio+"/SETUP/configuraciones.json"
            ruta_estado_scan = self.cargar_datos(ruta_configuraciones, "Path-estado-scan")
            f = open(ruta_estado_scan, "w")
            f.write(mensaje)
            f.close()
        except Exception as e:
            self.escribirArchivoLog("Error Escribir Archivo post_archivo_txt: "+str(e))
    
    def post_mediciones_txt(self, name : str,path = path):
        try:
            f = open(path+'/'+name+'.txt', "w")

            self.TOTALMR_AP = round(self.MRCAMION_AP + self.MRCARRO_AP,2)
            
            self.MR_TOTAL_E = round(self.MR_CAMION_E + self.MR_CARRO_E,2)
            
            self.MR_TOTAL_LC = round(self.MR_CAMION_LC + self.MR_CARRO_LC,2)
    
            mensaje = ("<--PLAN-B-->\n"+
                        "ALTURA_CAMION="+str(self.ALTURA_CAMION_E)+"\n"+
                        "LARGO_CAMION="+str(self.LARGO_CAMION_E)+"\n"+
                        "BASE_1_CAMION="+str(self.BASE_1_CAMION_E)+"\n"+
                        "BASE_2_CAMION="+str(self.BASE_2_CAMION_E)+"\n"+
                        "ANCHO_CAMION="+str(self.ANCHO_CAMION_E)+"\n"+
                        "MR_CAMION="+str(self.MR_CAMION_E)+"\n"+
                        "ALTURA_CARRO="+str(self.ALTURA_CARRO_E)+"\n"+
                        "LARGO_CARRO="+str(self.LARGO_CARRO_E)+"\n"+
                        "BASE_1_CARRO="+str(self.BASE_1_CARRO_E)+"\n"+
                        "BASE_2_CARRO="+str(self.BASE_2_CARRO_E)+"\n"+
                        "ANCHO_CAMION="+str(self.ANCHO_CARRO_E)+"\n"+
                        "MR_CARRO="+str(self.MR_CARRO_E)+"\n"+
                        "MR_TOTAL="+str(self.MR_TOTAL_E)+"\n"+
                        "<--PLAN-C-->\n"+
                        "VELOCIDAD_PROMEDIO_TOTAL="+str(self.VELOCIDAD_PROMEDIO_TOTAL)+"KM/H\n"+
                        "ALTURA_CAMION="+str(self.ALTURA_CAMION_LC)+"\n"+
                        "LARGO_CAMION="+str(self.LARGO_CAMION_LC)+"\n"+
                        "ANCHO_CAMION="+str(self.ANCHO_CAMION_LC)+"\n"+
                        "BASE_1_CAMION="+str(self.BASE_1_CAMION_LC)+"\n"+
                        "BASE_2_CAMION="+str(self.BASE_2_CAMION_LC)+"\n"+
                        "MR_CAMION="+str(self.MR_CAMION_LC)+"\n"+
                        "ALTURA_CARRO="+str(self.ALTURA_CARRO_LC)+"\n"+
                        "LARGO_CARRO="+str(self.LARGO_CARRO_LC)+"\n"+
                        "ANCHO_CARRO="+str(self.ANCHO_CARRO_LC)+"\n"+
                        "BASE_1_CARRO="+str(self.BASE_1_CARRO_LC)+"\n"+
                        "BASE_2_CARRO="+str(self.BASE_2_CARRO_LC)+"\n"+
                        "MR_CARRO="+str(self.MR_CARRO_LC)+"\n"+
                        "MR_TOTAL="+str(self.MR_TOTAL_LC)+"\n"+
                        "<--COMPARAR BASE-->\n"+ 
                        "BASE_1_CAMION="+str(self.BASE_1_CAMION_E)+"\n"+
                        "BASE_1_CAMION_INV="+str(self.BASE_1_CAMION_INV)+"\n"+
                        "BASE_2_CAMION="+str(self.BASE_2_CAMION_E)+"\n"+
                        "BASE_2_CAMION_INV="+str(self.BASE_2_CAMION_INV)+"\n"+
                        "BASE_1_CARRO="+str(self.BASE_1_CARRO_E)+"\n"+
                        "BASE_1_CARRO_INV="+str(self.BASE_1_CARRO_INV)+"\n"+
                        "BASE_2_CARRO="+str(self.BASE_2_CARRO_E)+"\n"+
                        "BASE_2_CARRO_INV="+str(self.BASE_2_CARRO_INV)+"\n"+
                        "<--COMPARAR ANCHO-->\n" +
                        "DISTANCIA_CAMION_BOTTOM="+str(self.DISTANCIA_CAMION_BOTTOM)+"\n"+
                        "DISTANCIA_CARRO_BOTTOM="+str(self.DISTANCIA_CARRO_BOTTOM)+"\n"+
                        "DISTANCIA_CAMION_RIGHT="+str(self.DISTANCIA_CAMION_RIGHT)+"\n"+
                        "DISTANCIA_CARRO_RIGHT="+str(self.DISTANCIA_CARRO_RIGHT)+"\n"+
                        "ANCHO_CAMION_CALCULADO="+str(self.ANCHO_CAMION_CALCULADO)+"\n"+
                        "ANCHO_CARRO_CALCULADO="+str(self.ANCHO_CARRO_CALCULADO)+"\n")
            f.write(mensaje)
            f.close()
        except Exception as e:
            self.escribirArchivoLog("Error Escribir archivo Mediciones.txt: "+str(e))

    def post_mediciones_default(self, name : str,path = path):
        try:
            f = open(path+'/'+name+'.txt', "w")

            self.TOTALMR_AP = round(self.MRCAMION_AP + self.MRCARRO_AP,2)
            
            self.MR_TOTAL_E = round(self.MR_CAMION_E + self.MR_CARRO_E,2)
            
            self.MR_TOTAL_LC = round(self.MR_CAMION_LC + self.MR_CARRO_LC,2)
    
            mensaje = ("<--PLAN-B-->\n"+
                            "ALTURA_CAMION=0\n"+
                            "LARGO_CAMION=0\n"+
                            "BASE_1_CAMION=0\n"+
                            "BASE_2_CAMION=0\n"+
                            "ANCHO_CAMION=0\n"+
                            "MR_CAMION=0\n"+
                            "ALTURA_CARRO=0\n"+
                            "LARGO_CARRO=0\n"+
                            "BASE_1_CARRO=0\n"+
                            "BASE_2_CARRO=0\n"+
                            "ANCHO_CAMION=0\n"+
                            "MR_CARRO=0\n"+
                            "MR_TOTAL=0\n"+
                            "<--PLAN-C-->\n"+
                            "VELOCIDAD_PROMEDIO_TOTAL="+str(self.VELOCIDAD_PROMEDIO_TOTAL)+"KM/H\n"+
                            "ALTURA_CAMION=0\n"+
                            "LARGO_CAMION=0\n"+
                            "ANCHO_CAMION=2.4\n"+
                            "BASE_1_CAMION=1.42\n"+
                            "BASE_2_CAMION=1.42\n"+
                            "MR_CAMION=1\n"+
                            "ALTURA_CARRO=0\n"+
                            "LARGO_CARRO=0\n"+
                            "ANCHO_CARRO=2.4\n"+
                            "BASE_1_CARRO=1.42\n"+
                            "BASE_2_CARRO=1.42\n"+
                            "MR_CARRO=1\n"+
                            "MR_TOTAL=2"+
                            "<--COMPARAR BASE-->\n"+ 
                            "BASE_1_CAMION=0\n"+
                            "BASE_1_CAMION_INV=0\n"+
                            "BASE_2_CAMION=0\n"+
                            "BASE_2_CAMION_INV=0\n"+
                            "BASE_1_CARRO=0\n"+
                            "BASE_1_CARRO_INV=0\n"+
                            "BASE_2_CARRO=0\n"+
                            "BASE_2_CARRO_INV=0\n"+
                            "<--COMPARAR ANCHO-->\n" +
                            "DISTANCIA_CAMION_BOTTOM=0\n"+
                            "DISTANCIA_CARRO_BOTTOM=0\n"+
                            "DISTANCIA_CAMION_RIGHT=0\n"+
                            "DISTANCIA_CARRO_RIGHT=0\n"+
                            "ANCHO_CAMION_CALCULADO=0\n"+
                            "ANCHO_CARRO_CALCULADO=0\n")
            f.write(mensaje)
            f.close()
        except Exception as e:
            self.escribirArchivoLog("Error Escribir archivo Mediciones.txt: "+str(e))
    
    def write_txt(self,path, mensaje : str):
        try:
            f = open(path, "w")
            f.write(mensaje)
            f.close()
        except Exception as e:
            self.escribirArchivoLog("Error Escribir Archivo write_txt: "+str(e))

    def enviar_mensaje_correo(self,mensaje):
        try:
            sender_email = "ttc.albertopernalete@gmail.com"
            password = "alberto18412"
            hoy = datetime.today()
            fecha = hoy.strftime("%Y-%m-%d %H:%M:%S")

            message = MIMEMultipart("alternative")
            message["Subject"] = "TTC_SCAN "+fecha
            message["From"] = "TTC_SCAN ARAUCO LAS CRUCES <"+sender_email+">"
            message["To"] = "ttc.albertopernalete@gmail.com"
            message["CC"] = "ttc.alejandracatejo@gmail.com, cvillegasttc@gmail.com"
            # message["Bcc"] = "cvillegasttc@gmail.com"

            # Create the plain-text and HTML version of your message
            text = mensaje

            # Turn these into plain/html MIMEText objects
            part1 = MIMEText(text, "plain")

            # Add HTML/plain-text parts to MIMEMultipart message
            # The email client will try to render the last part first
            message.attach(part1)

            # Create secure connection with server and send email
            context = ssl.create_default_context()
            with smtplib.SMTP_SSL("smtp.gmail.com", 465, context=context) as server:
                server.login(sender_email, password)
                # server.sendmail(
                #     sender_email, "ttc.albertopernalete@gmail.com", message.as_string()
                # )
                server.send_message(message)
                server.quit()
        except Exception as e:
            self.escribirArchivoLog("Error enviando correo eléctronico: "+str(e))

    def write_serial(self, dato : str):
        try:
            # Encender Rojo = S21
            # Apagar Rojo = S20
            # Encender Verde = S31
            # Apagar Verde = S30
            #Ruta archivos configuraciones.json
            ruta_configuraciones = "E:/TTC/TTC_SCAN/SETUP/configuraciones.json"
            #Ruta todos los camiones
            port = self.cargar_datos(ruta_configuraciones,"Port")
            baudrate = self.cargar_datos(ruta_configuraciones,"Baudrate")
            timeout = self.cargar_datos(ruta_configuraciones,"Timeout")
            control_semaforo = self.cargar_datos(ruta_configuraciones,"CONTROL-SEMAFORO")

            if control_semaforo:
                ser = serial.Serial(
                                port=port, 
                                baudrate = baudrate,
                                parity=serial.PARITY_NONE,
                                stopbits=serial.STOPBITS_ONE,
                                bytesize=serial.EIGHTBITS,
                                timeout=timeout
                        )
                sms = dato.encode()
                # print(sms)
                ser.write(sms)
                ser.write(chr(13).encode())
        except Exception as e:
            self.escribirArchivoLog("Error Escribir Serial Encender: "+str(e))
          
    def read_serial(self) -> str:
        try:
            # Encender Rojo = S21
            # Apagar Rojo = 20
            # Encender Verde = 31
            # Apagar Verde = 30
            #Ruta archivos configuraciones.json
            ruta_configuraciones = "E:/TTC/TTC_SCAN/SETUP/configuraciones.json"
            #Ruta todos los camiones
            port = self.cargar_datos(ruta_configuraciones,"Port")
            baudrate = self.cargar_datos(ruta_configuraciones,"Baudrate")
            timeout = self.cargar_datos(ruta_configuraciones,"Timeout")
            control_semaforo = self.cargar_datos(ruta_configuraciones,"CONTROL-SEMAFORO")
            string = ''

            if control_semaforo:
                ser = serial.Serial(
                                port=port, 
                                baudrate = baudrate,
                                parity=serial.PARITY_NONE,
                                stopbits=serial.STOPBITS_ONE,
                                bytesize=serial.EIGHTBITS,
                                timeout=timeout
                        )
                bin = ser.readline()
                # print(bin)
                string = bin.decode('utf-8').rstrip("\r\n")

            return string
        except Exception as e:
            self.escribirArchivoLog("Error leer Serial: "+str(e))

    def check_disco(self, unidad : str) -> int:
        hdd = psutil.disk_usage(unidad)

        total = round(hdd.total / (2**30))
        Used = round(hdd.used / (2**30))
        Free = round(hdd.free / (2**30))

        print(unidad,Free)
        
        return Free

    def post_espacio_disponible(self, name : str, path = path):
        try: 
            hoyHora = datetime.today()
            formatoHoraFecha = "%d/%m/%Y-%H:%M:%S"
            fecha_hora = hoyHora.strftime(formatoHoraFecha)
            operador_espacio = self.check_disco('S:/')
            scan_espacio = self.check_disco('E:/')
            
            f = open(path+name+'.txt', "w")    
            mensaje = ("SCAN="+str(scan_espacio)+"GB\n"+
                        "OPERADOR="+str(operador_espacio)+"GB\n"+
                        fecha_hora)
            f.write(mensaje)
            f.close()
        except Exception as e:
            self.escribirArchivoLog("Error Escribir archivo Mediciones.txt: "+str(e))
 