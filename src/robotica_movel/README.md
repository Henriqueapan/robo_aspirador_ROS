# Robótica Móvel - ROS Noetic

Este repositório contém pacotes relacionados ao projeto **professor_teste**, utilizado para simulações em ROS com o Gazebo. O projeto integra o modelo **Pioneer P3DX** com um ambiente de simulação.

## 🚀 Instalação e Configuração

### 🔹 1. Clonar o Repositório
Primeiro, certifique-se de que você tem um workspace do ROS configurado em `~/catkin_ws`. Caso não tenha, crie um com:

```bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
```

Agora, clone este repositório:

```bash
git clone https://github.com/IagoBiundini/robotica_movel.git
```

E entre no diretório do projeto:

```bash
cd robotica_movel
```

### 🔹 2. Instalar Dependências
Antes de compilar, instale as dependências do ROS usando `rosdep`:

```bash
cd ~/catkin_ws
rosdep install --from-paths src --ignore-src -r -y
```

Se encontrar erros, execute:

```bash
rosdep update
```

E tente novamente.

### 🔹 3. Compilar o Projeto
Agora, compile o workspace:

```bash
cd ~/catkin_ws
catkin_make
```

E ative o ambiente:

```bash
source devel/setup.bash
```

## 📦 Pacotes no Repositório

O repositório contém os seguintes pacotes:

### 🔹 **professor_teste (Meta-package)**
Este é um **meta-package** que organiza os pacotes do projeto.

### 🔹 **professor_mundo**
Pacote responsável pelo ambiente de simulação no Gazebo. Ele contém:
- O **mundo corredor** (`mundo_corredor.world`).
- Arquivos de configuração para o Gazebo.
- Scripts de controle e inicialização.

### 🔹 **Integração com o Pioneer P3DX**
O robô simulado no projeto é baseado no modelo **Pioneer P3DX**, disponível no repositório:

🔗 [pioneer_p3dx_model](https://github.com/mario-serna/pioneer_p3dx_model)

Para garantir que o modelo do robô funcione corretamente, o projeto carrega automaticamente os pacotes necessários para a simulação.

---

## 🏁 Executando a Simulação

### **1. Iniciar o Gazebo com o Mundo Corrredor**
Para iniciar o Gazebo com o ambiente **mundo_corredor**, execute:

```bash
roslaunch professor_mundo corredor.launch
```

Isso abrirá o Gazebo com o ambiente de teste.

---

## 🔧 Dúvidas ou Problemas?
Caso tenha dúvidas ou encontre problemas, sinta-se à vontade para abrir uma **Issue** no repositório.

📌 **Repositório:** [robotica_movel](https://github.com/IagoBiundini/robotica_movel)

🚀 **Bons testes e boas simulações!** 🤖🔥
