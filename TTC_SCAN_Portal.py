#!/usr/bin/env python
# -*- coding: utf-8 -*-
#LIbrerias especiales
import json
#Librerias generales
import time
from datetime import datetime
import psutil
import math
import itertools
from enum import IntEnum
from os import makedirs, remove
from os.path import exists, join
import glob
import subprocess
import shutil
#Libreria de TTC
from TTC_SCAN_CAMARAS import TTC_PORTAL as TTC_P

#------------------------------------
#----------INSTANCIAR TTC------------
#------------------------------------
ttc = TTC_P()

#Si el semaforo está encendido, se apaga.
ttc.write_serial(ttc.apagar_rojo)
ttc.write_serial(ttc.apagar_verde)

#Iniciar el Lidar siempre
ttc.write_serial(ttc.encender_lidar)


ischeck = ttc.check_cameras_on()
if ischeck == False:
    ttc.enviar_mensaje_correo("Inicio del SISTEMA TTC_SCAN ARAUCO CON CÁMARAS OFF")
    ttc.post_archivo_txt("RESET-OFF")
elif ischeck == True:
    ttc.enviar_mensaje_correo("Inicio del SISTEMA TTC_SCAN ARAUCO CON CÁMARAS ON")

ruta_directorio = ttc.obtener_ruta_actual()
ruta_configuraciones = ruta_directorio+"/SETUP/configuraciones.json"
if exists(ttc.cargar_datos(ruta_configuraciones,"Path-Modo")):
    ttc.escribirArchivoModo("f")

ttc.post_archivo_txt("ESPERANDO")
validador_nsemaforo = True

while True:
    sensor_cabina = ''
    if exists(ttc.cargar_datos(ruta_configuraciones,"Path-Modo")):
        try:
            modo = ttc.leerArchivoModo(ttc.cargar_datos(ruta_configuraciones,"Path-Modo"),1)
        except Exception as e:
            ttc.escribirArchivoLog("Error Leer Archivo Modo: "+str(e))  

        try:
            nsemaforo = ttc.leerArchivo(ttc.cargar_datos(ruta_configuraciones,"Path-nsemaforo"))
        except Exception as e:
            ttc.escribirArchivoLog("Error Leer Archivo nsemaforo: "+str(e))

        control_semaforo = ttc.cargar_datos(ruta_configuraciones,"CONTROL-SEMAFORO")

        #print(nsemaforo)
        if nsemaforo == "NEGRO":

                
            if validador_nsemaforo == False:
                ttc.write_serial(ttc.apagar_rojo)
                validador_nsemaforo = True
            
            #print(nsemaforo)
            if control_semaforo == True:
                try:
                    datos_modulos = ttc.read_serial()
                    if datos_modulos != '':
                        #print(datos_modulos)
                        datos_sensores = datos_modulos.split(' ')[1]
                        datos_reley = datos_modulos.split(' ')[2]
                        estado_luz_roja = datos_reley[3]
                        # print(datos_sensores)
                        sensor_cabina = datos_sensores[1]
                        # print(sensor_cabina)            
            

                    if sensor_cabina == '0':
                        # print("No Camión")
                        if estado_luz_roja == '1':
                            ttc.write_serial(ttc.apagar_rojo)
                        else:
                            pass
                    elif sensor_cabina == '':
                        pass
                    else: 
                        time.sleep(1)
                        if sensor_cabina == '1':
                            ttc.write_serial(ttc.encender_rojo)
                except Exception as e:
                    sensor_cabina = ''
                    ttc.escribirArchivoLog("Error Comunicación Serial Semáforo: "+str(e))


        elif nsemaforo == "ROJO":
            
            #print(nsemaforo)
            if validador_nsemaforo == True:
                ttc.write_serial(ttc.encender_rojo)
                validador_nsemaforo = False

        else:
            pass

        if modo == "a":
            # Apagar
            break
            #subprocess.call("shutdown -s")
        
        elif modo == "t": 
            ttc.post_archivo_txt("LUZ-ROJA") 
            ttc.write_serial(ttc.encender_rojo)
            ttc.reiniciar_Variables()
            time.sleep(2)

            validar_disco_ttcscan = ttc.cargar_datos(ruta_configuraciones,"ESPACIO-TTCSCAN")
            minimo_espacio_ttcscan = ttc.cargar_datos(ruta_configuraciones,"MINIMO-ESPACIO-TTCSCAN")
            validar_disco_operador = ttc.cargar_datos(ruta_configuraciones,"ESPACIO-OPERADOR")
            minimo_espacio_operador = ttc.cargar_datos(ruta_configuraciones,"MINIMO-ESPACIO-OPERADOR")
            espacio_libre_ttcscan = 0
            espacio_libre_operador = 0
            
            if validar_disco_ttcscan == True:
                espacio_libre_ttcscan = ttc.check_disco('E:/')
                if espacio_libre_ttcscan > minimo_espacio_ttcscan:
                    ttc.escribirArchivoLog("ESPACIO EN DISCO DEL TTCSCAN SUFICIENTE")
                else:
                    ttc.escribirArchivoLog("ESPACIO EN DISCO DEL TTCSCAN INSUFICIENTE")
                    ttc.enviar_mensaje_correo("ESPACIO EN DISCO DEL TTCSCAN INSUFICIENTE")

            if validar_disco_operador == True:
                espacio_libre_operador = ttc.check_disco('S:/')
                if espacio_libre_operador > minimo_espacio_operador:
                    ttc.escribirArchivoLog("ESPACIO EN DISCO DEL OPERADOR SUFICIENTE")
                else:
                    ttc.escribirArchivoLog("ESPACIO EN DISCO DEL OPERADOR INSUFICIENTE")
                    ttc.enviar_mensaje_correo("ESPACIO EN DISCO DEL OPERADOR INSUFICIENTE")

            
            #===================================
            #============INICIO SCAN============
            #===================================
            ttc.crearCarpetaData()   
            # Aqui se ejecuta la toma de fotos del DVR para Entrada y PatenteCamion
            
            FOTOS_DVR = ttc.cargar_datos(ruta_configuraciones,"FOTOS-DVR")
            if FOTOS_DVR == True:
                ttc.TomarFotoDVR('2', 'PatenteCamion')
                # ttc.TomarFotoDVR('3', 'PatenteCarro')
                ttc.TomarFotoDVR('1', 'FotoEntrada')
                # ttc.TomarFotoDVR('7', 'FotoSalida')
             
            try:              
                EjeX, EjeY, EjeZ,   \
                EjeX2, EjeY2, EjeZ2,    \
                EjeX3, EjeY3, EjeZ3,    \
                EjeX4, EjeY4, EjeZ4,    \
                tiempo_datos_por_linea = ttc.iniciarRecoleccionDeDistancias()
            except Exception as e:
                ttc.escribirArchivoLog("Error iniciarRecoleccionDeDistancias(): "+str(e))
                ttc.escribirArchivoModo("f")

        elif modo == "g":
            try:
                #=====Guardar datos en archivos=====      
                RS_TOP_ACTIVE = ttc.cargar_datos(ruta_configuraciones,"RS-TOP-ACTIVE")
                RS_CORNER_ACTIVE = ttc.cargar_datos(ruta_configuraciones,"RS-CORNER-ACTIVE")
                RS_BOTTOM_ACTIVE = ttc.cargar_datos(ruta_configuraciones,"RS-BOTTOM-ACTIVE")
                RS_RIGHT_ACTIVE = ttc.cargar_datos(ruta_configuraciones,"RS-RIGHT-ACTIVE")
                LIDAR_ACTIVE = ttc.cargar_datos(ruta_configuraciones,"Lidar-ACTIVE")

                if RS_TOP_ACTIVE == True:
                    datos = {}
                    datos['x'] = ttc.EjeX
                    datos['y'] = ttc.EjeY
                    datos['z'] = ttc.EjeZ
                    ttc.crear_json('Puntos-Top', datos)
                    ttc.escribirArchivo(str(ttc.EjeX)+"\n","-Top")
                    ttc.escribirArchivo(str(ttc.EjeY)+"\n","-Top")
                    ttc.escribirArchivo(str(ttc.EjeZ)+"\n","-Top")
                    ttc.contadorX = 1              
                else:
                    pass

                if RS_CORNER_ACTIVE == True:
                    datos2 = {}
                    datos2['x'] = ttc.EjeX2
                    datos2['y'] = ttc.EjeY2
                    datos2['z'] = ttc.EjeZ2
                    ttc.crear_json( 'Puntos-Corner', datos2)
                    ttc.escribirArchivo(str(ttc.EjeX2)+"\n","-Corner")
                    ttc.escribirArchivo(str(ttc.EjeY2)+"\n","-Corner")
                    ttc.escribirArchivo(str(ttc.EjeZ2)+"\n","-Corner")
                    ttc.contadorX2 = 1           
                else:
                    pass

                if RS_BOTTOM_ACTIVE == True:
                    datos3 = {}
                    datos3['x'] = ttc.EjeX3
                    datos3['y'] = ttc.EjeY3
                    datos3['z'] = ttc.EjeZ3
                    ttc.crear_json( 'Puntos-Bottom', datos3)
                    ttc.escribirArchivo(str(ttc.EjeX3)+"\n","-Bottom")
                    ttc.escribirArchivo(str(ttc.EjeY3)+"\n","-Bottom")
                    ttc.escribirArchivo(str(ttc.EjeZ3)+"\n","-Bottom")
                    ttc.contadorX3 = 1            
                else:
                    pass

                if RS_RIGHT_ACTIVE == True:
                    datos4 = {}
                    datos4['x'] = ttc.EjeX4
                    datos4['y'] = ttc.EjeY4
                    datos4['z'] = ttc.EjeZ4
                    ttc.crear_json( 'Puntos-Right', datos4)
                    ttc.escribirArchivo(str(ttc.EjeX4)+"\n","-Right")
                    ttc.escribirArchivo(str(ttc.EjeY4)+"\n","-Right")
                    ttc.escribirArchivo(str(ttc.EjeZ4)+"\n","-Right")
                    ttc.contadorX4 = 1            
                else:
                    pass
                #=====Guardar datos en archivos=====  
                    


                #--Se genera archivo para Carlos con todos los datos para Sync
                ttc.generated_file_full(ttc.tiempo_datos_por_linea, ttc.EjeZ, ttc.EjeZ2, ttc.EjeZ3)
                #--Se copia la info de NPesaje que Daniela genera
                ttc.post_lines_in_file(ttc.get_lines_in_file(ttc.cargar_datos(ruta_configuraciones,'Path-Pesaje')),'Datos-Camion',ttc.path) 
                
                #Se vacian [] las listas de los puntos de distancias RS y Lidar
                ttc.EjeX = []
                ttc.EjeY = []
                ttc.EjeZ = []
                ttc.EjeX2 = []
                ttc.EjeY2 = []
                ttc.EjeZ2 = []
                ttc.EjeX3 = []
                ttc.EjeY3 = []
                ttc.EjeZ3 = []
                ttc.EjeX4 = []
                ttc.EjeY4 = []
                ttc.EjeZ4 = []
                ttc.tiempo_datos_por_linea = []
                validador_nsemaforo = True
                #===================================
                #=============FIN SCAN==============
                #===================================
                #--Aqui el archivo Modo debe pasarse
                #--a la acción de medir
                MEDIR = ttc.cargar_datos(ruta_configuraciones,"MEDIR")
                if MEDIR == True:
                    ttc.escribirArchivoModo("m")
                else:
                    ttc.escribirArchivoModo("f")
            except Exception as e:
                ttc.escribirArchivoLog("Error al guardar archivos: "+str(e))

        elif modo == "m":
            #===================================
            #==========INICIO MEDICIÓN==========
            #===================================

            #1- Cargamos en un diccionario con la info de NPesaje.txt
            dict_npesaje = ttc.get_lines_in_file_dict(ttc.cargar_datos(ruta_configuraciones,'Path-Pesaje'))
            # print("******  dict_npesaje=",dict_npesaje)
            ttc.BANCOSCAMION = dict_npesaje['BANCOSCAMION']
            ttc.BANCOSCARRO = dict_npesaje['BANCOSCARRO']
            ttc.GUIA = dict_npesaje['GUIA']
            # tipo_bancos_carro = dict_npesaje['BANCOSCARRO']
            # tipo_bancos_camion = dict_npesaje['BANCOSCAMION']
            ttc.AUTOCARGANTE = dict_npesaje['AUTOCARGANTE']
            auto_cargante = ttc.AUTOCARGANTE 


            try:
                # print("MIDIENDO")
                ttc.post_archivo_txt("MIDIENDO")

                #2- Cargamos el factor de corrección de configuraciones.json
                #Factor corrección
                #factor_correcion = ttc.cargar_datos(ruta_configuraciones,"Factor_Correccion")
                #Factor corrección aplica o no
                #aplica_factor = ttc.cargar_datos(ruta_configuraciones,"Aplica_Factor")
                ruta_camiones = ttc.cargar_datos(ruta_configuraciones,"Ruta-Camiones")
                ruta_camion = ruta_camiones+"/"+ dict_npesaje['IDPESAJE']
                ttc.PES_ID = dict_npesaje['IDPESAJE']
                ttc.IDMEDICION = dict_npesaje['IDMEDICION']
                # print(ttc.PES_ID)
                ttc.MP_PATENTE = dict_npesaje['PATENTE']
                if 'PATENTECARRO' in dict_npesaje:
                    ttc.MP_PATENTE_CARRO = dict_npesaje['PATENTECARRO']
                else:
                    ttc.MP_PATENTE_CARRO = dict_npesaje['PATENTE']
                #print(ruta_camion)

                #3- Cargar los datos de los archivos json del camión y preparar las listas
                lista_original_Top_z = ttc.cargar_datos(ruta_camion+'/Puntos-Top.json','z')
                lista_original_Top_x = ttc.cargar_datos(ruta_camion+'/Puntos-Top.json','x')
                lista_original_Top_z = ttc.regenerete_list_z(lista_original_Top_z)
                lista_original_Top_x = ttc.regenerete_list_z(lista_original_Top_x)
                lista_top_z = []
                lista_top_x = []
                centro_puntos = ttc.cargar_datos(ruta_configuraciones,"PUNTO-CENTRO")
                # print(centro_puntos)

                for i, top in enumerate(lista_original_Top_z):
                    lista_top_z.append(top[centro_puntos])
                    lista_top_x.append(lista_original_Top_x[i][centro_puntos])

                # print(lista_original_Top_z[0].count(0.0))
                # print(lista_original_Top_z[10].count(0.0))
                # print(lista_original_Top_z[15].count(0.0))


                #4- Ejecutar obtener_camion_carro
                lista_indice = ttc.obtener_camion_carro(lista_top_z)
                # print(lista_indice)


                #5- Detectar inicio camion y fin carro
                inicio_camion = lista_indice[0]
                fin_carro = lista_indice[-1]
                # print(inicio_camion,fin_carro)

                #6-Calcular Velocidad completa del camión
                velocidad_camion = ttc.obtener_velocidad_camion(inicio_camion,fin_carro,ruta_camion)
                # print("Velocidad:",str(velocidad_camion)+"km")

                lista_camion_carro_z = lista_top_z[inicio_camion:fin_carro]
                lista_camion_carro_x = lista_top_x[inicio_camion:fin_carro]
                    
                #6- Filtrar la cabina de todos los camiones
                camion_z_final,  camion_x_final = ttc.filtrar_cabina(lista_original_Top_z, 
                                                                    inicio_camion, 
                                                                    fin_carro, 
                                                                    lista_top_z[inicio_camion:fin_carro], 
                                                                    lista_top_x[inicio_camion:fin_carro], 
                                                                    lista_top_z, 
                                                                    lista_top_x)
                # print(camion_z_final,  camion_x_final)

                #7- Detectar Fin Camión e Inicio de Carro
                fin_camion, inicio_carro = ttc.separar_camion_carro(lista_original_Top_z,
                                                                    camion_z_final,
                                                                    camion_x_final,
                                                                    ruta_camion,
                                                                    camion_x_final[0],
                                                                    fin_carro,
                                                                    inicio_camion)
                                
                    
                # print("Inicio Camion",inicio_camion)
                # print("Inicio B1", camion_x_final[0])
                # print("Fin Camion",fin_camion)
                # print("Inicio Carro",inicio_carro)
                # print("Fin Carro",fin_carro)
          





            #**** MEDICION ESTEREO CAMION ****
                if auto_cargante == 'NO':
                    try:
                        lista_graficar_camion = ttc.medicion_full_estereo('CAMION',
                                                        camion_x_final[0],
                                                        fin_camion,
                                                        lista_original_Top_z,
                                                        lista_top_z,
                                                        lista_top_x,
                                                        dict_npesaje,
                                                        velocidad_camion,
                                                        ruta_camion)
                        if ttc.MR_CAMION_E > 0 or ttc.MR_CAMION_LC > 0:
                            estado_estereo_camion = True
                        else:
                            estado_estereo_camion = False
                    except Exception as e:
                        estado_estereo_camion = False
                else:
                    try: 
                        #print("SI Autocargante")
                        lista_graficar_autocargante = ttc.medicion_full_estereo_autocargante('CAMION',
                                                                                    camion_x_final[0],
                                                                                    fin_camion,
                                                                                    lista_original_Top_z,
                                                                                    lista_top_z,
                                                                                    lista_top_x,
                                                                                    dict_npesaje,
                                                                                    velocidad_camion,
                                                                                    ruta_camion)
                        if ttc.MR_CAMION_E > 0 or ttc.MR_CAMION_LC > 0:
                            estado_estereo_camion = True
                        else:
                            estado_estereo_camion = False

                    except Exception as e:
                        estado_estereo_camion = False
            #**** FIN MEDICION ESTEREO CAMION ****
            
            #**** MEDICION ESTEREO CARRO ****
                try:
                    lista_graficar_carro = ttc.medicion_full_estereo('CARRO',
                                                    inicio_carro,
                                                    fin_carro,
                                                    lista_original_Top_z,
                                                    lista_top_z,
                                                    lista_top_x,
                                                    dict_npesaje,
                                                    velocidad_camion,
                                                    ruta_camion)
                    if ttc.MR_CARRO_E > 0 or ttc.MR_CARRO_LC > 0:
                        estado_estereo_carro = True
                    else:
                        estado_estereo_carro = False
                except Exception as e:
                    estado_estereo_carro = False
            #**** MEDICION ESTEREO CARRO ****


            #**** GRAFICAR LATERALES ****
                try:
                    ttc.post_graficar_3D_lateral(ttc.lista_graficar_camion_bottom, 
                                                    ttc.lista_graficar_carro_bottom, 
                                                    'bottom')
                    ttc.post_graficar_3D_lateral(ttc.lista_graficar_camion_right, 
                                                    ttc.lista_graficar_carro_right, 
                                                    'right')
                except Exception as e:
                    ttc.escribirArchivoLog("Error Guardar gráficas laterales: "+str(e))
            #**** FIN GRAFICAR LATERALES ****

                
            #**** GUARDAR GRÁFICAs ****
                try:
                    if auto_cargante == 'SI' and estado_estereo_camion == True:
                        ttc.post_graficar_3D(lista_graficar_autocargante, lista_graficar_carro)
                        #ttc.post_graficar_2D()
                    elif auto_cargante == 'NO' and estado_estereo_camion == True:
                        #ttc.post_graficar_3D()
                        ttc.post_graficar_3D(lista_graficar_camion, lista_graficar_carro)
                        #ttc.post_graficar_2D()
                except Exception as e:
                    ttc.escribirArchivoLog("Error Guardar gráficas: "+str(e))
            #**** FIN GUARDAR GRÁFICAs **** 

                ttc.post_mediciones_txt('Mediciones-'+str(ttc.PES_ID),ruta_camion)

            
            #ACTIVADO Inicio Guardar BD PLAN B
                #Se intenta guardar en la BD
                try:
                    #Si se calculó bien el PLAN B para el camipon y carro
                    if estado_estereo_camion == True and estado_estereo_carro == True:

                        print("Listo para guardar en DB...")
                        guardar_datos_db = ttc.cargar_datos(ruta_configuraciones, "guardar-datos-db")
                        print("guardar_datos_db",guardar_datos_db)
                        #Se consulta la configuración si se guarda en BD o no
                        if guardar_datos_db == True:
                            ttc.guardar_db_estereo()        
                            print("Guardado correctamente en DB Estereo ======> OK") 
                        else:
                            print("Desactivado guardar en DB Estereo") 
                    else:
                        #En caso de que salga mal alguna medición se guarda en BD el PLAN C
                        ttc.guardar_db_estereo_lc()
                        print("Error al intentar guardar en DB Estereo LC")
                except Exception as e:
                    try:
                        #En caso de que de error guardar en BD se guarda los datos por defecto
                        #Total MR camion 2MR
                        ttc.guardar_db_estereo_defecto()  
                    except Exception as e: 
                        ttc.escribirArchivoLog("Error no Existe camión en BD: "+str(e))
                    ttc.escribirArchivoLog("Error Guardar BD: "+str(e))
            #Fin Guardar BD PLAN B

            #DESACTIVADO Inicio Guardar BD PLAN C
                """print("Listo para guardar en DB...")
                guardar_datos_db = ttc.cargar_datos(ruta_configuraciones, "guardar-datos-db")
                print("guardar-datos-db",guardar_datos_db)
                try:
                    if guardar_datos_db == True:
                        ttc.guardar_db_estereo_lc()        
                        print("Guardado correctamente en DB Estereo ======> OK") 
                    else:
                        print("No se guarda en la BD") 
                except Exception as e:
                    try:
                        ttc.guardar_db_estereo_defecto()  
                    except Exception as e: 
                        ttc.escribirArchivoLog("Error no Existe camión en DB: "+str(e))
                    ttc.escribirArchivoLog("Error Guardar DB: "+str(e))"""
            #Fin Guardar BD PLAN C
                   
                
                #===================================
                #=============FIN MEDIR==============
                #===================================
                #--Aqui el archivo Modo debe pasarse
                #--a la acción de exportar
                EXPORTAR = ttc.cargar_datos(ruta_configuraciones,"EXPORTAR")
                if EXPORTAR == True:
                    ttc.escribirArchivoModo("e")
                else:                    
                    ttc.post_archivo_txt("EXPORTANDO")
                    print("Exportando...")
                    print("Exportado ======> OK")
                    ttc.post_archivo_txt("EXPORTADO")
                    time.sleep(3)
                    ttc.post_archivo_txt("ESPERANDO")
                    ttc.reiniciar_Variables()
                    ttc.escribirArchivoModo("f")

            except Exception as e:
                ttc.post_mediciones_default('Mediciones-'+str(ttc.PES_ID),ruta_camion)
                try:
                    ttc.guardar_db_estereo_defecto()  
                except Exception as f: 
                    ttc.escribirArchivoLog("Error no Existe camión en DB: "+str(f))
                ttc.escribirArchivoLog("Error Flujo Principal: "+str(e))
                ttc.escribirArchivoModo("e")
            #===================================
            #===========FIN MEDICIÓN============
            #===================================
            
        elif modo == "e":
            #===================================
            #==========INICIO EXPORTAR==========
            #===================================
            #Crear carpeta en PC Romanero
            
            ttc.post_archivo_txt("EXPORTANDO")
            print("Exportando...")
            ttc.crearCarpetaExportar()
            #Copiar los archivos en PC Romanero
            ttc.copiarArchivosExportar()
            print("Exportado ======> OK")
            ttc.post_estado_txt(ruta_camion, "Estado", "FINALIZADO")
            ttc.post_espacio_disponible("MEMORIA-DISCO", "S:/")
            isOkHR = ttc.realsense_hardware_reset()
            print('IsOkHR',isOkHR)
            if isOkHR == True:
                ttc.escribirArchivoLog("Hardaware Reset al final de medición OK")
            else:
                ttc.escribirArchivoLog("Error del Hardaware Reset al final de medición")
                #Reset por modulo
                ttc.write_serial(ttc.reset_PC)
                #Reset por Raspberry 
                #ttc.write_txt("R:/TTC_SCAN/Modo.txt","t")

            ttc.reiniciar_Variables()
            ttc.post_archivo_txt("EXPORTADO")
            time.sleep(3)
            print("ESPERANDO")
            ttc.post_archivo_txt("ESPERANDO")
            ttc.escribirArchivoModo("f")
            #===================================
            #===========FIN EXPORTAR============
            #===================================

        elif modo == "c":
            ttc.crearCarpetaData()
            ttc.escribirArchivoModo("f")

        elif modo == "f":
            #ttc.post_archivo_txt("ESPERANDO")
            pass

        elif modo == "r":
            pass
            # Reiniciar
            #subprocess.call("shutdown -r")

        elif modo == "h":
            ttc.post_archivo_txt("CONECTADO")
            time.sleep(2)
            ttc.post_archivo_txt("ESPERANDO")
            ttc.escribirArchivoModo("f")

        else:
            pass
    
    else:
        #print("Sin MODO")
        pass