# Pacote Robô Aspirador

Este pacote ROS contém um simulador de robô aspirador usando o Gazebo com um modelo Pioneer 3DX e um ambiente de casa personalizado.

## Estrutura do Pacote

```
robo_aspirador/
├── launch/
│   ├── robo_aspirador.launch    # Launch principal
│   └── test_robot.launch        # Launch para testes
├── scripts/
│   └── test_robot_control.py    # Script de teste
├── worlds/
│   ├── casa/                     # Modelo da casa (SDF)
│   │   ├── model.config
│   │   └── model.sdf
│   └── casa_mundo.world          # Mundo completo do Gazebo
├── CMakeLists.txt
└── package.xml
```

## Dependências

- ROS Noetic
- Gazebo Classic
- gazebo_ros
- p3dx_gazebo
- p3dx_description
- p3dx_control
- xacro

## Como Usar

### 1. Compilar o Workspace

```bash
cd /root/catkin_ws
catkin_make
source devel/setup.bash
```

### 2. Executar o Simulador

```bash
roslaunch robo_aspirador robo_aspirador.launch
```

Isso irá:
- Iniciar o Gazebo com o mundo da casa
- Spawnar o robô Pioneer 3DX
- Configurar o controle do robô

### 3. Testar o Controle do Robô

```bash
roslaunch robo_aspirador test_robot.launch
```

Este comando irá executar um teste automático que move o robô para frente e depois o faz girar.

### 4. Controle Manual por Teclado

Para controlar o robô manualmente com o teclado:

```bash
# Opção 1: Launch completo (simulador + controle)
roslaunch robo_aspirador teleop_control.launch

# Opção 2: Apenas o controle (se o simulador já estiver rodando)
rosrun robo_aspirador robo_aspirador_keyteleop.py
```

**Controles disponíveis:**
- `w/x`: mover para frente/trás
- `a/d`: girar esquerda/direita  
- `s`: parar
- `q`: modo limpeza automática
- `e`: voltar ao ponto inicial
- `r/f`: aumentar/diminuir velocidade
- `CTRL-C`: sair

### 5. Controle Manual Direto

Para controlar o robô manualmente via comandos ROS:

```bash
# Publicar comandos de velocidade
rostopic pub /p3dx/cmd_vel geometry_msgs/Twist "linear:
  x: 0.2
  y: 0.0
  z: 0.0
angular:
  x: 0.0
  y: 0.0
  z: 0.0"
```

### 6. Visualizar no RViz

```bash
roslaunch p3dx_description rviz.launch
```

## Tópicos Disponíveis

- `/p3dx/cmd_vel` - Comandos de velocidade
- `/p3dx/odom` - Odometria do robô
- `/p3dx/laser/scan` - Dados do laser
- `/p3dx/joint_states` - Estados das juntas
- `/gazebo/model_states` - Estados dos modelos no Gazebo

## Problemas Resolvidos

1. **Arquivo de mundo incorreto**: O launch original tentava carregar um arquivo `.sdf` como mundo, mas o Gazebo espera um arquivo `.world`.

2. **Argumentos incorretos**: Corrigido o argumento `name` no spawn do robô.

3. **Dependências faltando**: Adicionadas todas as dependências necessárias no `package.xml` e `CMakeLists.txt`.

4. **Spawn da casa**: Criado um arquivo de mundo completo que inclui a casa diretamente.

## Estrutura do Ambiente

O ambiente inclui:
- Um plano de fundo (ground_plane)
- Iluminação (sun)
- O modelo da casa com paredes e estruturas
- O robô Pioneer 3DX com sensores laser

## Documentação e Aprendizado

### Guia Completo de ROS
Para aprender mais sobre ROS, consulte o guia completo em:
```
docs/guia_ros_completo.md
```

Este guia inclui:
- Conceitos fundamentais do ROS
- Como criar nós, publishers e subscribers
- Services, Actions e Parâmetros
- Launch files e exemplos práticos
- Comandos úteis e boas práticas

## Próximos Passos

Para expandir este projeto, você pode:
- Adicionar mais móveis à casa
- Implementar algoritmos de navegação
- Adicionar sensores adicionais
- Criar mapas do ambiente
- Implementar comportamentos de limpeza
- Estudar o guia ROS para criar funcionalidades mais avançadas
