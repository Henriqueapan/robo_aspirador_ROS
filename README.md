# Como rodar

## No host

`xhost +local:docker`

## No container (exemplo de mundo vazio)

1. `roscore`
2. `roslaunch gazebo_ros empty_world.launch`
3. `roslaunch p3dx_gazebo p3dx.launch`
4. `rosrun begginner_tutorials teste.py`

## Rodar GMapping + RViz

Com roscore em execução:

1. `roslaunch robo_aspirador robo_aspirador.launch`
2. `rosrun robo_aspirador robo_aspirador_keyteleop.py`
3. `rviz`
4. Configurar um frame de Map e outro de LaserScan
4.1. Clicar em Add no canto inferior esquerdo
4.2. Para o Map, configurar o tópico /map
4.3. Para o LaserScan, configurar o tópico /p3dx/laser/scan

### Imagem gerada à partir do container executando gmapping e rviz adequadamente:

ros-gmapping-rviz