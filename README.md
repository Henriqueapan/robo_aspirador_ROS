# Robô Aspirador P3DX - Sistema de Autonavegação

## Como rodar

### No host

`xhost +local:docker`

### Correção de Terminações de Linha (Windows)

**A conversão de terminações de linha é feita automaticamente pelo launch file!** 

O sistema agora converte automaticamente os arquivos Python de CRLF (Windows) para LF (Unix) toda vez que você executa o launch file. Não é mais necessário fazer isso manualmente.

Se ainda encontrar o erro `/usr/bin/env: 'python3\r': No such file or directory`, você pode executar manualmente:

```bash
# Opção 1: Usando o script Python
python3 fix_line_endings.py

# Opção 2: Manualmente com sed
cd src/robo_aspirador/scripts
sed -i 's/\r$//' *.py
```

### Sistema de Autonavegação Completo

Para usar o sistema completo com modos de Mapeamento e Aspiração, com roscore em execução:

```bash
roslaunch robo_aspirador robo_aspirador_controller.launch
```

Isso iniciará:
- Gazebo com o mundo
- Robô P3DX
- GMapping para SLAM
- Controlador principal com menu interativo

**No menu, escolha:**
- `1` - Modo Mapeamento (controle manual para mapear o ambiente)
- `2` - Modo Aspiração (navegação automática usando A*)
- `q` - Sair

### Rodar GMapping + RViz (Modo Manual)

Com roscore em execução:

1. `roslaunch robo_aspirador robo_aspirador.launch`
2. `rosrun robo_aspirador robo_aspirador_keyteleop.py`
3. `rviz`
4. Configurar um frame de Map e outro de LaserScan
4.1. Clicar em Add no canto inferior esquerdo
4.2. Para o Map, configurar o tópico /map
4.3. Para o LaserScan, configurar o tópico /p3dx/laser/scan

### Exemplo de mundo vazio (sem o sistema completo)

1. `roscore`
2. `roslaunch gazebo_ros empty_world.launch`
3. `roslaunch p3dx_gazebo p3dx.launch`
4. `rosrun begginner_tutorials teste.py`

### Documentação

Para mais detalhes sobre o sistema de navegação, consulte:
- `src/robo_aspirador/README_NAVEGACAO.md` - Documentação completa do sistema

### Imagem gerada à partir do container executando gmapping e rviz adequadamente:

ros-gmapping-rviz