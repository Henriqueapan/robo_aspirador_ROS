#!/usr/bin/env python

# Importa o módulo rospy para trabalhar com ROS
import rospy
import random
from beginner_tutorials.msg import Ambiente

# Função principal que será executada pelo nó ROS
def sensor_node():
    # Cria um publicador no tópico 'dados_ambiente' que usa a mensagem Ambiente
    pub = rospy.Publisher('dados_ambiente', Ambiente, queue_size=10)
    # Inicializa o nó ROS com o nome 'sensor', garantindo que seja anônimo
    rospy.init_node('sensor_node', anonymous=True)
    # Define a taxa de publicação em 2Hz
    rate = rospy.Rate(0.5) 

  
    
    
   

    # Loop para percorrer todos os dados e publicar no tópico
    while not rospy.is_shutdown():
        medicoes = Ambiente()
        medicoes.temperatura = (20*random.random()+15)
        medicoes.umidade = (30*random.random()+40)               
        pub.publish(medicoes)
        rate.sleep()

# Ponto de entrada do programa
if __name__ == '__main__':
    try:
        sensor_node()
    except rospy.ROSInterruptException:
        pass
