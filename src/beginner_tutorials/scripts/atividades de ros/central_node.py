#!/usr/bin/env python
# Importa o módulo rospy para trabalhar com ROS
import rospy
from beginner_tutorials.msg import Ambiente

def callback(data, args):
    sector = args
    print(f"{sector} - Temperatura:{data.temperatura:.2f}°C, Umidade:{data.umidade:.2f}%")
    if data.temperatura >= 30:
        print(f"Alerta! Temperatura elevada no {sector}")
    if data.umidade >= 45:
        if data.umidade <= 65:
            print(f"Alerta! Umidadeforado intervalo no {sector}")
def central_node():
    rospy.init_node('central_node', anonymous=False)

    # Subscriptions para cada setor
    rospy.Subscriber('/setor_a/dados_ambiente', Ambiente, callback, "Setor A")
    rospy.Subscriber('/setor_b/dados_ambiente', Ambiente, callback, "Setor B")
    rospy.Subscriber('/setor_c/dados_ambiente', Ambiente, callback, "Setor C")

    rospy.spin()

if __name__ == '__main__':
    central_node()
