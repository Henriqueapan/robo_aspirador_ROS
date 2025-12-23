#!/usr/bin/env python3

import rospy
import sys
import os
import select
import termios
import tty
from geometry_msgs.msg import Twist

# Adiciona o diretório dos scripts ao path para importação
script_dir = os.path.dirname(os.path.abspath(__file__))
if script_dir not in sys.path:
    sys.path.insert(0, script_dir)

from robo_aspirador_keyteleop import RoboAspiradorTeleop
from navigation_planner import NavigationPlanner

class RoboAspiradorController:
    """
    Controlador principal do robô aspirador.
    Gerencia os modos: Mapeamento (manual) e Aspiração (automática).
    """
    
    MODE_MAPPING = 'mapping'
    MODE_VACUUM = 'vacuum'
    
    def __init__(self):
        rospy.init_node('robo_aspirador_controller', anonymous=True)
        
        # Publisher para comandos de velocidade (backup)
        self.cmd_vel_pub = rospy.Publisher('/p3dx/cmd_vel', Twist, queue_size=1)
        
        # Configurações do teclado
        self.settings = termios.tcgetattr(sys.stdin)
        
        # Instâncias dos modos
        self.teleop = None
        self.planner = None
        
        # Estado atual
        self.current_mode = None
        self.running = True
        
        # Mensagem de ajuda
        self.help_msg = """
        ============================================
        CONTROLADOR DO ROBÔ ASPIRADOR P3DX
        ============================================
        
        MODOS DISPONÍVEIS:
        
        1 - MODO MAPEAMENTO
           Controle manual do robô para mapear o ambiente.
           Use as teclas w/a/s/d/x para controlar o robô.
           Pressione 'q' para sair deste modo.
        
        2 - MODO ASPIRAÇÃO
           Navegação automática usando A* para percorrer
           todo o ambiente mapeado.
           Pressione 'q' para cancelar e sair.
        
        COMANDOS GLOBAIS:
        q - Sair do modo atual / Encerrar programa
        CTRL-C - Encerrar programa
        
        ============================================
        """
        
    def get_key(self):
        """Captura tecla pressionada"""
        tty.setraw(sys.stdin.fileno())
        select.select([sys.stdin], [], [], 0)
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key
    
    def stop_robot(self):
        """Para o robô"""
        twist = Twist()
        self.cmd_vel_pub.publish(twist)
    
    def enter_mapping_mode(self):
        """Entra no modo de mapeamento (controle manual)"""
        rospy.loginfo("=== ENTRANDO NO MODO MAPEAMENTO ===")
        rospy.loginfo("Use as teclas w/a/s/d/x para controlar o robô")
        rospy.loginfo("Pressione 'q' para sair deste modo")
        
        try:
            # Cria instância do teleop (sem inicializar nó ROS, pois já temos um)
            self.teleop = RoboAspiradorTeleop(
                velocidade_linear_max=1.0,
                velocidade_angular_max=1.0,
                node_name='',  # Não cria novo nó
                init_node=False,
                allow_exit_on_q=True  # Permite sair com 'q'
            )
            
            # Executa o loop de controle manual
            self.teleop.run()
            
        except Exception as e:
            rospy.logerr(f"Erro no modo mapeamento: {e}")
        finally:
            self.stop_robot()
            self.teleop = None
            rospy.loginfo("=== SAINDO DO MODO MAPEAMENTO ===")
    
    def enter_vacuum_mode(self):
        """Entra no modo de aspiração (navegação automática)"""
        rospy.loginfo("=== ENTRANDO NO MODO ASPIRAÇÃO ===")
        rospy.loginfo("Aguardando mapa...")
        rospy.loginfo("Pressione 'q' para cancelar")
        
        try:
            # Cria instância do planejador (sem inicializar nó ROS, pois já temos um)
            self.planner = NavigationPlanner(init_node=False)
            
            # Aguarda o mapa
            if not self.planner.wait_for_map(timeout=10.0):
                rospy.logerr("Mapa não disponível. Execute o mapeamento primeiro!")
                self.planner = None
                return
            
            rospy.loginfo("Mapa recebido! Iniciando cobertura...")
            
            # Executa cobertura em uma thread separada para permitir cancelamento
            import threading
            
            coverage_thread = threading.Thread(target=self._execute_coverage_thread)
            coverage_thread.daemon = True
            coverage_thread.start()
            
            # Loop para verificar cancelamento
            rate = rospy.Rate(10)
            cancelled = False
            while coverage_thread.is_alive() and not rospy.is_shutdown():
                # Verifica se há tecla pressionada (não bloqueante)
                if select.select([sys.stdin], [], [], 0)[0]:
                    key = self.get_key()
                    if key == 'q':
                        rospy.loginfo("Cancelando aspiração...")
                        cancelled = True
                        if self.planner:
                            self.planner.cancelled = True
                            self.planner.stop_robot()
                        break
                rate.sleep()
            
            # Aguarda thread terminar
            coverage_thread.join(timeout=1.0)
            
        except Exception as e:
            rospy.logerr(f"Erro no modo aspiração: {e}")
        finally:
            self.stop_robot()
            self.planner = None
            rospy.loginfo("=== SAINDO DO MODO ASPIRAÇÃO ===")
    
    def _execute_coverage_thread(self):
        """Executa cobertura em thread separada"""
        try:
            if self.planner:
                self.planner.execute_coverage(cell_size=0.5)
        except Exception as e:
            rospy.logerr(f"Erro na execução de cobertura: {e}")
    
    def show_menu(self):
        """Mostra menu de seleção de modo"""
        print("\n" + "="*50)
        print("SELECIONE O MODO:")
        print("="*50)
        print("1 - MODO MAPEAMENTO (Controle Manual)")
        print("2 - MODO ASPIRAÇÃO (Navegação Automática)")
        print("q - SAIR")
        print("="*50)
        print("\nDigite sua escolha: ", end='', flush=True)
    
    def run(self):
        """Loop principal do controlador"""
        try:
            print(self.help_msg)
            
            while self.running and not rospy.is_shutdown():
                self.show_menu()
                
                # Aguarda entrada do usuário
                key = self.get_key()
                print(key)  # Mostra a tecla pressionada
                
                if key == '1':
                    self.enter_mapping_mode()
                elif key == '2':
                    self.enter_vacuum_mode()
                elif key == 'q' or key == '\x03':  # q ou CTRL-C
                    rospy.loginfo("Encerrando controlador...")
                    break
                else:
                    print(f"\nOpção inválida: '{key}'. Tente novamente.")
                
        except KeyboardInterrupt:
            rospy.loginfo("Interrompido pelo usuário")
        except Exception as e:
            rospy.logerr(f"Erro no controlador: {e}")
        finally:
            self.stop_robot()
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
            print("\n\nPrograma encerrado. Robô parado.")

if __name__ == '__main__':
    try:
        controller = RoboAspiradorController()
        controller.run()
    except rospy.ROSInterruptException:
        print("\n\nPrograma interrompido pelo ROS.")

