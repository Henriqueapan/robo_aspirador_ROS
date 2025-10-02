#!/usr/bin/env python

from __future__ import print_function

from beginner_tutorials.srv import ControleIluminacao, ControleIluminacaoResponse
import rospy

def controle(requisito):
    global estado
    if requisito.comando in ['ligar', 'desligar','estado']:
        if estado == 0:
            if requisito.comando == 'ligar':
                estado = 1
                return ControleIluminacaoResponse(print(f"A Lâmpada foi ligada!"))
            if requisito.comando == 'desligar':
                return ControleIluminacaoResponse(print(f"A Lâmpada já está desligada!"))
        if estado == 1:
            if requisito.comando == 'ligar':
                return ControleIluminacaoResponse(print(f"A Lâmpada já está ligada!"))
            if requisito.comando == 'desligar':
                estado = 0
                return ControleIluminacaoResponse(print(f"A Lâmpada foi desligada!"))
        if requisito.comando == 'estado':
            if estado == 1:
                return ControleIluminacaoResponse(print(f"A Lâmpada está ligada!"))
            if estado == 0:
                return ControleIluminacaoResponse(print(f"A Lâmpada está desligada!"))
    else:
        return ControleIluminacaoResponse(print(f"Os comandos devem ser no formato 'ligar', 'desligar' ou 'estado'"))

def iluminacao_server():
    rospy.init_node('iluminacao_server')
    s = rospy.Service('iluminacao_server', ControleIluminacao, controle)
    rospy.spin()



if __name__ == "__main__":   
    global estado
    estado = 0
    iluminacao_server()
