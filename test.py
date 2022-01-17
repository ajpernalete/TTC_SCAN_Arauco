#!/usr/bin/env python
# -*- coding: utf-8 -*-

def function(arreglo:list, caracter : str):
    cont = 0
    res = ()

    for val in arreglo:
        if val == caracter:
            cont+=1

    res = (cont,len(arreglo))

    return res


res = function(["ab","a"],"ab")
print(res)