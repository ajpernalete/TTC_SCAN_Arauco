#!/usr/bin/env python
# -*- coding: utf-8 -*-
#LIbrerias especiales
import MySQLdb as mdb
import logging


def escribirArchivoLog(mensaje):
    logging.basicConfig(level=logging.DEBUG,
                    format='%(asctime)s ===> %(levelname)s ===> TTC_SCAN_Portal.exe ===> %(message)s',
                    filename = 'E:/TTC/TTC_SCAN/SETUP/Log-Errores.log',
                    filemode = 'a',)
    logging.info(mensaje)


def conexion_db(servidor,usuario,clave,nombre_db):
    try:
        db = mdb.connect(servidor, usuario, clave, nombre_db)
        if db:
            print("Conexión éxitosa")
            return db
    except Exception as e:
        print("Error conexión")
        escribirArchivoLog("Error Conexión Base de Datos: "+str(e))

# conec_db = conexion_db('DESKTOP-H7FLD3C', 'portal', 'pituca188', 'ttcpesaje') 

var  = "CAMION_A"
print(var, var[:-1])