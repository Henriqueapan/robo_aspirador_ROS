# Imagem base com ROS Noetic e Gazebo incluídos
FROM osrf/ros:noetic-desktop-full

# Evita prompts interativos
ENV DEBIAN_FRONTEND=noninteractive

# Atualiza e instala dependências e utilitários
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-rosdep \
    python3-rosinstall \
    python3-rosinstall-generator \
    python3-wstool \
    build-essential \
    git \
    nano \
    net-tools \
    iputils-ping \
    ros-noetic-slam-gmapping \
    ros-noetic-openslam-gmapping \
    ros-noetic-navigation \
    ros-noetic-map-server \
    && rm -rf /var/lib/apt/lists/*

# Instala dependências Python se quiser
RUN pip3 install rospkg catkin_pkg

# Inicializa rosdep (necessário para usar dependências)
RUN rosdep init || true
RUN rosdep update

# Cria um workspace para desenvolvimento
RUN mkdir -p /root/catkin_ws/src
WORKDIR /root/catkin_ws

# Copia o código em src necessário do projeto
COPY ./src/robo_aspirador /root/catkin_ws/src/robo_aspirador
COPY ./src/beginner_tutorials /root/catkin_ws/src/beginner_tutorials
COPY ./src/robotica_movel /root/catkin_ws/src/robotica_movel

# Clona o move_base_flex (branch noetic), tracking_pid e full_coverage_path_planner dentro de src
RUN git clone -b noetic https://github.com/naturerobots/move_base_flex.git /root/catkin_ws/src/move_base_flex \
    && git clone https://github.com/nobleo/tracking_pid.git /root/catkin_ws/src/tracking_pid \
    && git clone https://github.com/nobleo/full_coverage_path_planner.git /root/catkin_ws/src/full_coverage_path_planner

# Instala as dependências dos pacotes instalados em src
RUN rosdep install --from-paths src --ignore-src -r -y

# Compila o workspace vazio (apenas estrutura)
RUN /bin/bash -c "source /opt/ros/noetic/setup.bash && catkin_make"

# Seta o ambiente ao abrir o container
RUN echo 'source /opt/ros/noetic/setup.bash' >> /root/.bashrc \
 && echo 'source /root/catkin_ws/devel/setup.bash' >> /root/.bashrc

# Diretório de trabalho padrão
WORKDIR /root/catkin_ws/src
