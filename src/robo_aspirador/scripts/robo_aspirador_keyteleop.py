#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
import sys, select, termios, tty

class RoboAspiradorTeleop:
    def __init__(self, velocidade_linear_max = 1.0, velocidade_angular_max = 1.0):
        rospy.init_node('robo_aspirador_keyteleop')
        
        # Publisher para comandos de velocidade do P3DX
        self.pub = rospy.Publisher('/p3dx/cmd_vel', Twist, queue_size=1)
        
        # Configurações do teclado
        self.settings = termios.tcgetattr(sys.stdin)
        
        # Instruções específicas para o robô aspirador
        self.msg = """
        CONTROLE MANUAL POR TECLADO DO ROBÔ ASPIRADOR P3DX
        ==================================================
        
        MOVIMENTO:
           w
        a    s    d
           x
        
        w/x : mover para frente/trás
        a/d : girar esquerda/direita
        s   : PARAR
        
        COMANDOS ESPECIAIS:
        q   : modo limpeza automática
        e   : voltar ao ponto inicial
        r   : aumentar velocidade
        f   : diminuir velocidade
        
        CTRL-C para sair
        """
        
        # Configurações de velocidade
        self.velocidade_linear = 0.3
        self.velocidade_angular = 0.5
        self.velocidade_linear_max = velocidade_linear_max
        self.velocidade_angular_max = velocidade_angular_max
        self.velocidade_min = 0.1
        
    def get_key(self):
        """Captura tecla pressionada"""
        tty.setraw(sys.stdin.fileno())
        select.select([sys.stdin], [], [], 0)
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key
    
    def vels(self, speed, turn):
        """Retorna string com velocidades."""
        return f"Velocidade Linear: {speed:.2f} m/s | Velocidade Angular: {turn:.2f} rad/s"
    
    def modo_limpeza(self):
        """
        Simula modo de limpeza automática.
        Realiza alguns movimentos para simular uma limpeza do ambiente.
        """
        rospy.loginfo("Iniciando modo de limpeza automática...")
        
        # Padrão de movimento para limpeza
        padroes = [
            (0.2, 0.0, 2),    # Frente
            (0.0, 0.3, 1),    # Gira
            (0.2, 0.0, 2),    # Frente
            (0.0, -0.3, 1),   # Gira
        ]
        
        for linear, angular, tempo in padroes:
            if rospy.is_shutdown():
                break
                
            twist = Twist()
            twist.linear.x = linear
            twist.angular.z = angular
            self.pub.publish(twist)
            
            rospy.loginfo(f"Limpando... Linear: {linear}, Angular: {angular}")
            rospy.sleep(tempo)
        
        # Para o robô
        twist = Twist()
        self.pub.publish(twist)
        rospy.loginfo("Limpeza concluída.")


    def voltar_estacao_carga(self):
        """
        Simula retorno à estação de carga.
        Na prática, apenas simula o que seria o movimento de retornar para a estação de carga.
        Não volta de fato para um lugar pré-definido, apenas faz alguns movimentos simulando isso.
        """
        rospy.loginfo("Voltando à estação de carga...")
        
        # Simula movimento de retorno
        twist = Twist()
        twist.linear.x = -0.2  # Move para trás
        self.pub.publish(twist)
        rospy.sleep(3)
        
        # Para o robô
        twist = Twist()
        self.pub.publish(twist)
        rospy.loginfo("Chegou ao ponto inicial.")
    
    def run(self):
        """Loop principal de controle"""
        speed = 0
        turn = 0
        status = 0
        
        try:
            print(self.msg)
            print(self.vels(speed, turn))
            
            while not rospy.is_shutdown():
                key = self.get_key()
                
                # Movimento básico, sem composição de linear com angular
                if key == 'w':
                    speed = self.velocidade_linear
                    turn = 0
                elif key == 'x':
                    speed = -self.velocidade_linear
                    turn = 0
                elif key == 'a':
                    speed = 0
                    turn = self.velocidade_angular
                elif key == 'd':
                    speed = 0
                    turn = -self.velocidade_angular
                elif key == 's':
                    speed = 0
                    turn = 0
                
                # Controle de velocidade
                elif key == 'r': # Aumenta velocidade
                    self.velocidade_linear = min(self.velocidade_linear_max, self.velocidade_linear + 0.1)
                    self.velocidade_angular = min(self.velocidade_angular_max, self.velocidade_angular + 0.1)
                    rospy.loginfo(f"Velocidade aumentada: Linear={self.velocidade_linear:.2f}, Angular={self.velocidade_angular:.2f}")
                elif key == 'f': #Diminui velocidade
                    self.velocidade_linear = max(self.velocidade_min, self.velocidade_linear - 0.1)
                    self.velocidade_angular = max(self.velocidade_min, self.velocidade_angular - 0.1)
                    rospy.loginfo(f"Velocidade diminuída: Linear={self.velocidade_linear:.2f}, Angular={self.velocidade_angular:.2f}")
                
                # Modos especiais
                elif key == 'q':
                    self.modo_limpeza()
                elif key == 'e':
                    self.voltar_estacao_carga()
                
                # Sair
                elif key == '\x03':  # CTRL-C
                    break
                
                # Publica comando de movimento
                twist = Twist()
                twist.linear.x = speed
                twist.angular.z = turn
                self.pub.publish(twist)
                
                # Atualiza status
                if key != '' and key not in ['r', 'f', 'q', 'e']:
                    print(f"\r{self.vels(speed, turn)}", end='', flush=True)
                
        except Exception as e:
            print(f"\nErro: {e}")
        finally:
            # Para o robô
            twist = Twist()
            self.pub.publish(twist)
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
            print("\nRobô parado. Programa encerrado.")

if __name__ == '__main__':
    try:
        teleop = RoboAspiradorTeleop()
        teleop.run()
    except rospy.ROSInterruptException:
        print("\nPrograma interrompido pelo usuário.")