#!/usr/bin/env python3

import rospy
import numpy as np
from nav_msgs.msg import OccupancyGrid, MapMetaData
from geometry_msgs.msg import Twist, PoseStamped, Pose, Point, Quaternion
from tf import TransformListener
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import math
import heapq
from collections import deque

class NavigationPlanner:
    """
    Planejador de navegação usando A* para encontrar caminhos
    e algoritmo de cobertura para percorrer toda a área do mapa.
    """
    
    def __init__(self, init_node=True):
        if init_node:
            rospy.init_node('navigation_planner', anonymous=True)
        
        # Subscribers
        self.map_sub = rospy.Subscriber('/map', OccupancyGrid, self.map_callback)
        self.map_data = None
        self.map_metadata = None
        self.map_received = False
        
        # Publisher para comandos de velocidade
        self.cmd_vel_pub = rospy.Publisher('/p3dx/cmd_vel', Twist, queue_size=1)
        
        # TF listener para obter posição do robô
        self.tf_listener = TransformListener()
        self.base_frame = 'base_link'
        self.map_frame = 'map'
        
        # Parâmetros de controle
        self.linear_speed = 0.25  # Reduzido para movimento mais suave
        self.angular_speed = 0.4  # Reduzido para rotação mais suave
        self.position_tolerance = 0.2  # metros (aumentado para evitar oscilações)
        self.angle_tolerance = 0.15  # radianos (aumentado)
        
        # Grid de cobertura
        self.coverage_grid = None
        self.visited_cells = set()
        self.coverage_path = []
        self.current_goal_index = 0
        
        # Estado
        self.current_goal = None
        self.is_navigating = False
        self.cancelled = False
        
        rospy.loginfo("NavigationPlanner inicializado. Aguardando mapa...")
        
    def map_callback(self, msg):
        """Callback quando recebe o mapa"""
        self.map_data = np.array(msg.data).reshape((msg.info.height, msg.info.width))
        self.map_metadata = msg.info
        self.map_received = True
        rospy.loginfo(f"Mapa recebido: {msg.info.width}x{msg.info.height} pixels, resolução: {msg.info.resolution} m/pixel")
        
    def wait_for_map(self, timeout=10.0):
        """Aguarda o mapa ser recebido"""
        start_time = rospy.Time.now()
        while not self.map_received and not rospy.is_shutdown():
            if (rospy.Time.now() - start_time).to_sec() > timeout:
                rospy.logwarn("Timeout aguardando mapa")
                return False
            rospy.sleep(0.1)
        return True
    
    def world_to_map(self, x, y):
        """Converte coordenadas do mundo para coordenadas do mapa"""
        if self.map_metadata is None:
            return None, None
        
        map_x = int((x - self.map_metadata.origin.position.x) / self.map_metadata.resolution)
        map_y = int((y - self.map_metadata.origin.position.y) / self.map_metadata.resolution)
        
        return map_x, map_y
    
    def map_to_world(self, map_x, map_y):
        """Converte coordenadas do mapa para coordenadas do mundo"""
        if self.map_metadata is None:
            return None, None
        
        x = map_x * self.map_metadata.resolution + self.map_metadata.origin.position.x
        y = map_y * self.map_metadata.resolution + self.map_metadata.origin.position.y
        
        return x, y
    
    def is_valid_cell(self, x, y):
        """Verifica se uma célula é válida (livre) para navegação"""
        if self.map_data is None:
            return False
        
        if x < 0 or x >= self.map_metadata.width or y < 0 or y >= self.map_metadata.height:
            return False
        
        # 0 = livre, 100 = ocupado, -1 = desconhecido
        # Consideramos livre apenas células com valor <= 20 (margem de segurança)
        return self.map_data[y, x] <= 20
    
    def get_neighbors(self, x, y):
        """Retorna vizinhos válidos de uma célula (8-direções)"""
        neighbors = []
        for dx in [-1, 0, 1]:
            for dy in [-1, 0, 1]:
                if dx == 0 and dy == 0:
                    continue
                nx, ny = x + dx, y + dy
                if self.is_valid_cell(nx, ny):
                    neighbors.append((nx, ny))
        return neighbors
    
    def heuristic(self, x1, y1, x2, y2):
        """Heurística para A* (distância euclidiana)"""
        return math.sqrt((x1 - x2)**2 + (y1 - y2)**2)
    
    def a_star(self, start_x, start_y, goal_x, goal_y):
        """
        Implementação do algoritmo A* para encontrar caminho entre dois pontos.
        Retorna lista de células (x, y) do caminho ou None se não encontrar.
        """
        if not self.is_valid_cell(start_x, start_y) or not self.is_valid_cell(goal_x, goal_y):
            rospy.logwarn("Células inicial ou final inválidas")
            return None
        
        # Fila de prioridade: (f_score, g_score, x, y, parent)
        open_set = [(0, 0, start_x, start_y, None)]
        closed_set = set()
        g_scores = {(start_x, start_y): 0}
        parents = {}
        
        while open_set:
            f_score, g_score, x, y, parent = heapq.heappop(open_set)
            
            if (x, y) in closed_set:
                continue
            
            closed_set.add((x, y))
            parents[(x, y)] = parent
            
            if x == goal_x and y == goal_y:
                # Reconstrói o caminho
                path = []
                current = (x, y)
                while current is not None:
                    path.append(current)
                    current = parents.get(current)
                path.reverse()
                return path
            
            for nx, ny in self.get_neighbors(x, y):
                if (nx, ny) in closed_set:
                    continue
                
                # Custo do movimento (1 para ortogonal, sqrt(2) para diagonal)
                move_cost = 1.0 if abs(nx - x) + abs(ny - y) == 1 else math.sqrt(2)
                tentative_g = g_score + move_cost
                
                if (nx, ny) not in g_scores or tentative_g < g_scores[(nx, ny)]:
                    g_scores[(nx, ny)] = tentative_g
                    h_score = self.heuristic(nx, ny, goal_x, goal_y)
                    f_score = tentative_g + h_score
                    heapq.heappush(open_set, (f_score, tentative_g, nx, ny, (x, y)))
        
        rospy.logwarn("A* não encontrou caminho")
        return None
    
    def get_robot_pose(self):
        """Obtém a posição atual do robô no frame do mapa"""
        try:
            # Tenta obter a transformação com timeout menor para evitar bloqueios
            if not self.tf_listener.canTransform(self.map_frame, self.base_frame, rospy.Time(0)):
                self.tf_listener.waitForTransform(self.map_frame, self.base_frame, rospy.Time(), rospy.Duration(0.5))
            
            (trans, rot) = self.tf_listener.lookupTransform(self.map_frame, self.base_frame, rospy.Time(0))
            
            x, y = trans[0], trans[1]
            _, _, yaw = euler_from_quaternion(rot)
            
            return x, y, yaw
        except Exception as e:
            rospy.logwarn(f"Erro ao obter pose do robô: {e}")
            return None, None, None
    
    def generate_coverage_path(self, cell_size=0.5):
        """
        Gera um caminho de cobertura usando algoritmo de varredura (boustrophedon).
        cell_size: tamanho da célula de cobertura em metros.
        """
        if not self.map_received:
            rospy.logwarn("Mapa não recebido ainda")
            return []
        
        rospy.loginfo("Gerando caminho de cobertura...")
        
        # Converte cell_size para pixels do mapa
        cell_size_pixels = max(1, int(cell_size / self.map_metadata.resolution))
        
        coverage_path = []
        visited = set()
        
        # Gera grid de cobertura
        width = self.map_metadata.width
        height = self.map_metadata.height
        
        # Itera pelas linhas do grid de cobertura
        for row in range(0, height, cell_size_pixels):
            # Determina direção (esquerda->direita ou direita->esquerda)
            left_to_right = (row // cell_size_pixels) % 2 == 0
            
            if left_to_right:
                col_range = range(0, width, cell_size_pixels)
            else:
                col_range = range(width - 1, -1, -cell_size_pixels)
            
            for col in col_range:
                # Verifica se a célula central é válida
                if self.is_valid_cell(col, row):
                    # Verifica se há área navegável ao redor (margem de segurança)
                    margin = cell_size_pixels // 2
                    is_navigable = True
                    
                    # Verifica células ao redor para garantir que é área navegável
                    for dy in range(-margin, margin + 1):
                        for dx in range(-margin, margin + 1):
                            if not self.is_valid_cell(col + dx, row + dy):
                                is_navigable = False
                                break
                        if not is_navigable:
                            break
                    
                    # Adiciona se for área navegável
                    if is_navigable:
                        world_x, world_y = self.map_to_world(col, row)
                        if world_x is not None and world_y is not None:
                            coverage_path.append((world_x, world_y))
        
        # Remove pontos muito próximos e otimiza o caminho
        if len(coverage_path) > 1:
            filtered_path = [coverage_path[0]]
            min_distance = cell_size * 1.2  # Distância mínima aumentada entre pontos
            
            for point in coverage_path[1:]:
                last_point = filtered_path[-1]
                distance = math.sqrt((point[0] - last_point[0])**2 + (point[1] - last_point[1])**2)
                if distance >= min_distance:
                    filtered_path.append(point)
            
            # Se ainda houver muitos pontos, reduz ainda mais
            if len(filtered_path) > 100:
                rospy.loginfo(f"Reduzindo caminho de {len(filtered_path)} para pontos mais espaçados...")
                final_path = [filtered_path[0]]
                min_distance_final = cell_size * 2.0
                
                for point in filtered_path[1:]:
                    last_point = final_path[-1]
                    distance = math.sqrt((point[0] - last_point[0])**2 + (point[1] - last_point[1])**2)
                    if distance >= min_distance_final:
                        final_path.append(point)
                
                filtered_path = final_path
            
            coverage_path = filtered_path
        
        rospy.loginfo(f"Caminho de cobertura gerado com {len(coverage_path)} pontos")
        self.coverage_path = coverage_path
        return coverage_path
    
    def navigate_to_goal(self, goal_x, goal_y, goal_yaw=None):
        """
        Navega até um objetivo usando A* e controle simples.
        Retorna True quando chega ao objetivo.
        """
        if not self.map_received:
            rospy.logwarn("Mapa não recebido")
            return False
        
        # Obtém posição atual
        robot_x, robot_y, robot_yaw = self.get_robot_pose()
        if robot_x is None:
            return False
        
        # Converte para coordenadas do mapa
        start_map_x, start_map_y = self.world_to_map(robot_x, robot_y)
        goal_map_x, goal_map_y = self.world_to_map(goal_x, goal_y)
        
        if start_map_x is None or goal_map_x is None:
            rospy.logwarn("Erro ao converter coordenadas")
            return False
        
        # Calcula caminho com A*
        path = self.a_star(start_map_x, start_map_y, goal_map_x, goal_map_y)
        
        if path is None or len(path) == 0:
            rospy.logwarn("Não foi possível encontrar caminho")
            return False
        
        rospy.loginfo(f"Navegando para ({goal_x:.2f}, {goal_y:.2f}) via {len(path)} pontos")
        
        # Segue o caminho (pula alguns pontos intermediários para movimento mais fluido)
        step = max(1, len(path) // 20)  # Pega aproximadamente 20 pontos do caminho
        path_indices = list(range(0, len(path), step))
        if path_indices[-1] != len(path) - 1:
            path_indices.append(len(path) - 1)  # Garante que o último ponto está incluído
        
        rospy.loginfo(f"Navegando por {len(path_indices)} pontos do caminho de {len(path)} pontos totais")
        
        for idx, i in enumerate(path_indices):
            if rospy.is_shutdown() or self.cancelled:
                return False
            
            map_x, map_y = path[i]
            target_x, target_y = self.map_to_world(map_x, map_y)
            
            # Calcula ângulo desejado
            if i < len(path) - 1:
                next_idx = min(i + step, len(path) - 1)
                next_map_x, next_map_y = path[next_idx]
                next_x, next_y = self.map_to_world(next_map_x, next_map_y)
                target_yaw = math.atan2(next_y - target_y, next_x - target_x)
            elif goal_yaw is not None:
                target_yaw = goal_yaw
            else:
                # Obtém pose atual para manter orientação
                current_x, current_y, current_yaw = self.get_robot_pose()
                target_yaw = current_yaw if current_yaw is not None else None
            
            rospy.loginfo(f"Ponto {idx+1}/{len(path_indices)}: ({target_x:.2f}, {target_y:.2f})")
            
            # Navega até este ponto
            if not self.move_to_point(target_x, target_y, target_yaw):
                rospy.logwarn(f"Falha ao chegar ao ponto {idx+1}/{len(path_indices)}")
                # Continua para o próximo ponto em vez de retornar False
                continue
        
        rospy.loginfo("Chegou ao objetivo!")
        return True
    
    def move_to_point(self, target_x, target_y, target_yaw=None):
        """
        Move o robô até um ponto usando controle simples.
        Retorna True quando chega ao ponto.
        """
        rate = rospy.Rate(20)  # 20 Hz para controle mais suave
        max_iterations = 500  # Limite de iterações para evitar loops infinitos
        iteration = 0
        last_distance = float('inf')
        stuck_counter = 0
        
        while not rospy.is_shutdown() and not self.cancelled and iteration < max_iterations:
            iteration += 1
            robot_x, robot_y, robot_yaw = self.get_robot_pose()
            if robot_x is None:
                rospy.logwarn("Não foi possível obter pose do robô")
                rospy.sleep(0.1)
                continue
            
            # Calcula distância e ângulo até o objetivo
            dx = target_x - robot_x
            dy = target_y - robot_y
            distance = math.sqrt(dx**2 + dy**2)
            angle_to_goal = math.atan2(dy, dx)
            
            # Detecta se está preso (não está se aproximando)
            if abs(distance - last_distance) < 0.01 and distance > self.position_tolerance:
                stuck_counter += 1
                if stuck_counter > 50:  # Preso por mais de 2.5 segundos
                    rospy.logwarn(f"Robô pode estar preso. Distância: {distance:.2f}m")
                    # Tenta uma manobra de escape
                    twist = Twist()
                    twist.angular.z = 0.3
                    self.cmd_vel_pub.publish(twist)
                    rospy.sleep(0.5)
                    stuck_counter = 0
            else:
                stuck_counter = 0
            
            last_distance = distance
            
            # Verifica se chegou
            if distance < self.position_tolerance:
                # Se há orientação alvo, gira para ela
                if target_yaw is not None:
                    angle_error = target_yaw - robot_yaw
                    # Normaliza ângulo para [-pi, pi]
                    while angle_error > math.pi:
                        angle_error -= 2 * math.pi
                    while angle_error < -math.pi:
                        angle_error += 2 * math.pi
                    
                    if abs(angle_error) > self.angle_tolerance:
                        twist = Twist()
                        # Velocidade angular proporcional ao erro
                        twist.angular.z = max(0.1, min(self.angular_speed, abs(angle_error) * 0.5)) * (1 if angle_error > 0 else -1)
                        self.cmd_vel_pub.publish(twist)
                        rate.sleep()
                        continue
                
                # Para o robô
                self.stop_robot()
                rospy.loginfo(f"Chegou ao ponto! Distância final: {distance:.3f}m")
                return True
            
            # Calcula erro angular
            angle_error = angle_to_goal - robot_yaw
            # Normaliza ângulo para [-pi, pi]
            while angle_error > math.pi:
                angle_error -= 2 * math.pi
            while angle_error < -math.pi:
                angle_error += 2 * math.pi
            
            twist = Twist()
            
            # Controlador proporcional melhorado
            # Primeiro gira para o objetivo, mas permite movimento simultâneo se o erro for pequeno
            angle_threshold = 0.4  # radianos (~23 graus) - aumentado para permitir movimento mais cedo
            
            if abs(angle_error) > angle_threshold:
                # Erro grande: apenas gira (mas com velocidade controlada)
                angular_vel = max(0.15, min(self.angular_speed, abs(angle_error) * 0.6))
                twist.angular.z = angular_vel * (1 if angle_error > 0 else -1)
            else:
                # Erro pequeno: pode mover e girar simultaneamente
                # Velocidade linear proporcional à distância e ao alinhamento
                linear_factor = min(1.0, distance / 0.8)  # Reduz velocidade quando próximo
                angular_factor = abs(angle_error) / angle_threshold  # Reduz rotação quando alinhado
                
                # Garante movimento linear mesmo com pequeno erro angular
                twist.linear.x = max(0.1, self.linear_speed * linear_factor * (1.0 - angular_factor * 0.2))
                twist.angular.z = self.angular_speed * angular_factor * 0.5 * (1 if angle_error > 0 else -1)
            
            self.cmd_vel_pub.publish(twist)
            rate.sleep()
        
        if iteration >= max_iterations:
            rospy.logwarn(f"Timeout ao tentar chegar ao ponto ({target_x:.2f}, {target_y:.2f})")
        
        self.stop_robot()
        return False
    
    def stop_robot(self):
        """Para o robô"""
        twist = Twist()
        self.cmd_vel_pub.publish(twist)
    
    def execute_coverage(self, cell_size=0.5):
        """
        Executa a cobertura completa do ambiente.
        Retorna True quando completa, False se cancelado.
        """
        self.cancelled = False
        
        if not self.map_received:
            rospy.logwarn("Aguardando mapa...")
            if not self.wait_for_map():
                return False
        
        # Gera caminho de cobertura
        if not self.coverage_path:
            self.generate_coverage_path(cell_size)
        
        if not self.coverage_path:
            rospy.logwarn("Não foi possível gerar caminho de cobertura")
            return False
        
        rospy.loginfo(f"Iniciando cobertura com {len(self.coverage_path)} pontos")
        
        # Obtém posição inicial
        start_x, start_y, _ = self.get_robot_pose()
        if start_x is not None:
            rospy.loginfo(f"Posição inicial do robô: ({start_x:.2f}, {start_y:.2f})")
        
        # Navega por cada ponto
        successful_points = 0
        failed_points = 0
        
        for i, (x, y) in enumerate(self.coverage_path):
            if rospy.is_shutdown() or self.cancelled:
                rospy.loginfo("Cobertura cancelada")
                self.stop_robot()
                return False
            
            rospy.loginfo(f"=== Ponto {i+1}/{len(self.coverage_path)}: ({x:.2f}, {y:.2f}) ===")
            
            # Verifica se o ponto é acessível antes de tentar navegar
            map_x, map_y = self.world_to_map(x, y)
            if map_x is not None and not self.is_valid_cell(map_x, map_y):
                rospy.logwarn(f"Ponto {i+1} não é acessível, pulando...")
                failed_points += 1
                continue
            
            if not self.navigate_to_goal(x, y):
                if self.cancelled:
                    rospy.loginfo("Navegação cancelada")
                    self.stop_robot()
                    return False
                rospy.logwarn(f"Falha ao chegar ao ponto {i+1}, continuando...")
                failed_points += 1
                continue
            
            successful_points += 1
            rospy.loginfo(f"✓ Ponto {i+1} alcançado com sucesso! ({successful_points} sucessos, {failed_points} falhas)")
            
            if self.cancelled:
                rospy.loginfo("Cobertura cancelada")
                self.stop_robot()
                return False
            
            # Pequena pausa para "aspirar"
            rospy.sleep(0.3)
        
        rospy.loginfo("Cobertura completa finalizada!")
        self.stop_robot()
        return True

if __name__ == '__main__':
    try:
        planner = NavigationPlanner()
        
        if planner.wait_for_map():
            planner.execute_coverage()
        else:
            rospy.logerr("Não foi possível obter o mapa")
            
    except rospy.ROSInterruptException:
        pass

