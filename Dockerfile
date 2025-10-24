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
    && rm -rf /var/lib/apt/lists/*

# Instala dependências Python se quiser
RUN pip3 install rospkg catkin_pkg

# Inicializa rosdep (necessário para usar dependências)
RUN rosdep init || true
RUN rosdep update

# Cria um workspace para desenvolvimento
RUN mkdir -p /root/catkin_ws/src
WORKDIR /root/catkin_ws

# Compila o workspace vazio (apenas estrutura)
RUN /bin/bash -c "source /opt/ros/noetic/setup.bash && catkin_make"

# Seta o ambiente ao abrir o container
RUN echo 'source /opt/ros/noetic/setup.bash' >> /root/.bashrc \
 && echo 'source /root/catkin_ws/devel/setup.bash' >> /root/.bashrc

# Diretório de trabalho padrão
WORKDIR /root/catkin_ws/src
