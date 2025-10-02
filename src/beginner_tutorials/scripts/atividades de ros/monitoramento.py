#!/usr/bin/env python
# Importa o módulo rospy para trabalhar com ROS
import rospy
# Importa o tipo de mensagem padrão 'String' (embora não seja usado no código atual)
from std_msgs.msg import String
# Importa a mensagem customizada 'Funcionarios' do pacote 'beginner_tutorials'
from beginner_tutorials.msg import Funcionarios

# Função de callback que será chamada sempre que uma nova mensagem for recebida no tópico
def callback(data):
    # Exibe no terminal as informações do funcionário recebidas
    print('O nome do funcionario é:', data.nome, 
          'Idade:', data.idade, 
          'anos com o Cargo:', data.cargo, 
          'e Altura de:', data.altura, 'm')

# Função principal que configura o nó ROS e o assinante
def listener():
    # Inicializa o nó ROS com o nome 'listener', garantindo que seja anônimo
    rospy.init_node('listener', anonymous=True)
    
    # Cria um assinante no tópico 'dados_funcionarios' que usa a mensagem Funcionarios
    # Quando uma nova mensagem é recebida, a função callback é executada
    rospy.Subscriber('dados_funcionarios', Funcionarios, callback)
    
    # Mantém o nó em execução, ouvindo as mensagens do tópico
    rospy.spin()

# Ponto de entrada do programa
if __name__ == '__main__':
    # Executa a função listener
    listener()
