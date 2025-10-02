#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
import numpy as np
import math
from tf.transformations import euler_from_quaternion

class PotentialFieldNavigator:
    def __init__(self):
        rospy.init_node('potential_field_navigator', anonymous=True)
        
        # Publisher para enviar comandos de velocidade
        self.pub = rospy.Publisher('/p3dx/cmd_vel', Twist, queue_size=10)
        
        # Subscribers para receber informações do robô
        self.sub_odom = rospy.Subscriber('/p3dx/odom', Odometry, self.odometry_callback)
        self.sub_scan = rospy.Subscriber('/p3dx/laser/scan', LaserScan, self.laserscan_callback)
        
        print("[INFO] Subscribers para /p3dx/odom e /p3dx/scan iniciados.")
        
        # Ponto objetivo (x, y)
        self.goal = (9.0, 0.0)  # Definir um objetivo fixo
        self.goal_tolerance = 0.2  # O robô para quando estiver a menos de 20 cm do objetivo

        self.k1 = 0.3  # Constante do campo de atração
        self.k2 = 0.47  # Constante do campo de repulsão
        self.min_distance = .1  # Distância mínima para repulsão
        self.max_distance = 1.2  # Distância máxima para repulsão

        self.position = None
        self.yaw = 0.0
        self.ranges = []
        self.angle_min = 0.0
        self.angle_increment = 0.0

        rospy.sleep(2)  # Tempo para garantir que os tópicos estejam publicando

        print("[INFO] Iniciando controle do robô...")
        
        rate = rospy.Rate(10)  # 10 Hz
        while not rospy.is_shutdown():
            if self.position is not None:
                dist_to_goal = math.hypot(self.goal[0] - self.position.x, self.goal[1] - self.position.y)
                print(f"[DEBUG] Distância para o objetivo: {dist_to_goal:.2f} m")

                if dist_to_goal > self.goal_tolerance:  # Continua se estiver fora da tolerância
                    print("[DEBUG] Chamando control_robot()")
                    self.control_robot()
                else:
                    print("[INFO] Objetivo atingido. Parando o robô.")
                    self.stop_robot()
            else:
                print("[WARNING] Aguardando dados de odometria...")
            
            rate.sleep()

    def odometry_callback(self, msg):
        self.position = msg.pose.pose.position
        ori = msg.pose.pose.orientation
        
        # Converte quaternion para ângulo de Euler (yaw)
        quaternion = (ori.x, ori.y, ori.z, ori.w)
        euler = euler_from_quaternion(quaternion)
        self.yaw = euler[2]

    def laserscan_callback(self, msg):
        self.ranges = msg.ranges
        self.angle_min = msg.angle_min
        self.angle_increment = msg.angle_increment

    def calculate_attractive_force(self):
        if self.position is None:
            return 0, 0
        
        dx = self.goal[0] - self.position.x
        dy = self.goal[1] - self.position.y
        distance_to_goal = math.hypot(dx, dy)
        angle_to_goal = math.atan2(dy, dx)
        force_attractive = self.k1 * distance_to_goal
        
        
        
        return force_attractive, angle_to_goal, dx, dy

    def calculate_repulsive_force(self):
        
        force_x = 0.0
        force_y = 0.0
        
        for i, dist in enumerate(self.ranges):
            if dist < self.max_distance:  # Considera obstáculos dentro do alcance
                angle = self.angle_min + i * self.angle_increment  # Ângulo do obstáculo
    
                # Se o obstáculo estiver muito perto, força máxima
                if dist < self.min_distance:
                    repulsion = self.k2 * (1 / (self.min_distance ** 2))
                else:
                    repulsion = self.k2 * (1 / (dist ** 2))

                # Decomposição em componentes X e Y
                force_x -= repulsion * math.cos(angle)  # Direção oposta ao obstáculo
                force_y -= repulsion * math.sin(angle)

        # Cálculo do módulo e ângulo do vetor repulsivo resultante
        force_repulsive = math.hypot(force_x, force_y)  # Módulo da força
        angle_repulsive = math.atan2(force_y, force_x)  # Direção da força resultante

        force_repulsive = min(force_repulsive, 1000)  # Limita a força máxima para evitar bloqueio
        


        return force_repulsive, angle_repulsive, force_x, force_y
           

    # Evitar divisão por zero e calcular média ponderada do ângulo de repulsão
        if count > 0:
            angle_repulsive /= force_repulsive

        force_repulsive = min(force_repulsive, 1000)  # Limita a repulsão para evitar travamento
        drx = np.sin(angle_repulsive)*force_repulsive
        dry = np.cos(angle_repulsive)*force_repulsive


        return force_repulsive, angle_repulsive, drx, dry


    def control_robot(self):
        if self.position is None or len(self.ranges) == 0:
            print("[ERRO] Sem dados de odometria ou LaserScan!")
            return

        force_attractive, angle_attractive, dx, dy = self.calculate_attractive_force()
        force_repulsive, angle_repulsive, drx, dry = self.calculate_repulsive_force()

        total_force = max(0.25, force_attractive - force_repulsive)
        total_angle = angle_attractive + angle_repulsive

        max_angular_speed = 0.3
        angular_error = total_angle - self.yaw
        angular_control = max(-max_angular_speed, min(max_angular_speed, angular_error))

        if abs(angular_control) > 0.5:
            linear_speed = 0.8
        else:
            linear_speed = min(total_force, 0.8)

        print(f"[DEBUG] Publicando -> linear: {linear_speed:.2f}, angular: {angular_control:.2f}, atrativa: {force_attractive:.2f}, repulsiva: {force_repulsive:.2f}")
        print("força atrativa em x: ", dx,"força atrativa em y: ", dy)
        print("força repulsiva em x: ", drx,"força repulsiva em y: ", dry )
        
        

        vel = Twist()
        vel.linear.x = linear_speed
        vel.angular.z = angular_control
        self.pub.publish(vel)

    def stop_robot(self):
        vel = Twist()
        vel.linear.x = 0.0
        vel.angular.z = 0.0
        self.pub.publish(vel)
        print("[DEBUG] Comando de parada enviado.")

if __name__ == '__main__':
    try:
        navigator = PotentialFieldNavigator()
        rospy.spin()
    except rospy.ROSInterruptException:
        print("[INFO] Navegação finalizada.")

