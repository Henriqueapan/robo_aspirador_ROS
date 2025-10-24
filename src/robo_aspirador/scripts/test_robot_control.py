#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist

def test_robot_control():
    """Testa o controle do robô P3DX"""
    rospy.init_node('test_robot_control')
    
    # Publicador para comandos de velocidade
    cmd_vel_pub = rospy.Publisher('/p3dx/cmd_vel', Twist, queue_size=1)
    
    rospy.loginfo("Testando controle do robô P3DX...")
    
    # Aguarda um pouco para garantir que o robô está pronto
    rospy.sleep(2)
    
    # Comando para mover o robô para frente
    twist = Twist()
    twist.linear.x = 0.2  # Velocidade linear de 0.2 m/s
    twist.angular.z = 0.0  # Sem rotação
    
    rospy.loginfo("Movendo robô para frente por 3 segundos...")
    cmd_vel_pub.publish(twist)
    rospy.sleep(3)
    
    # Para o robô
    twist.linear.x = 0.0
    twist.angular.z = 0.0
    cmd_vel_pub.publish(twist)
    
    rospy.loginfo("Robô parado. Teste concluído!")
    
    # Comando para girar o robô
    twist.angular.z = 0.5  # Velocidade angular de 0.5 rad/s
    rospy.loginfo("Girando robô por 2 segundos...")
    cmd_vel_pub.publish(twist)
    rospy.sleep(2)
    
    # Para o robô novamente
    twist.angular.z = 0.0
    cmd_vel_pub.publish(twist)
    
    rospy.loginfo("Teste de controle concluído com sucesso!")

if __name__ == '__main__':
    try:
        test_robot_control()
    except rospy.ROSInterruptException:
        rospy.loginfo("Teste interrompido pelo usuário")
