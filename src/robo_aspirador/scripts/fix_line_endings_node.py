#!/usr/bin/env python3
"""
Nó ROS para converter terminações de linha automaticamente.
Executado pelo launch file antes de iniciar os outros nós.
"""

import rospy
import os
import sys

def fix_line_endings(filepath):
    """Converte CRLF para LF em um arquivo"""
    try:
        with open(filepath, 'rb') as f:
            content = f.read()
        
        # Verifica se precisa conversão
        if b'\r' not in content:
            return False  # Já está correto
        
        # Remove \r
        content = content.replace(b'\r\n', b'\n').replace(b'\r', b'\n')
        
        # Garante que termina com \n
        if not content.endswith(b'\n'):
            content += b'\n'
        
        with open(filepath, 'wb') as f:
            f.write(content)
        
        rospy.loginfo(f"[fix_line_endings] Convertido: {filepath}")
        return True
    except Exception as e:
        rospy.logwarn(f"[fix_line_endings] Erro ao converter {filepath}: {e}")
        return False

def main():
    rospy.init_node('fix_line_endings', anonymous=True, log_level=rospy.INFO)
    
    # Encontra o diretório de scripts
    try:
        script_dir = rospy.get_param('~script_dir', None)
        if script_dir is None:
            # Tenta encontrar usando rospack
            import subprocess
            result = subprocess.run(['rospack', 'find', 'robo_aspirador'], 
                                  capture_output=True, text=True)
            if result.returncode == 0:
                script_dir = os.path.join(result.stdout.strip(), 'scripts')
        else:
            script_dir = os.path.join(script_dir, 'scripts')
    except:
        # Fallback: caminho relativo
        script_dir = os.path.join(os.path.dirname(__file__))
    
    if not os.path.exists(script_dir):
        rospy.logwarn(f"[fix_line_endings] Diretório não encontrado: {script_dir}")
        rospy.signal_shutdown("Diretório não encontrado")
        return
    
    files = [
        'robo_aspirador_controller.py',
        'navigation_planner.py',
        'robo_aspirador_keyteleop.py'
    ]
    
    converted_count = 0
    for filename in files:
        filepath = os.path.join(script_dir, filename)
        if os.path.exists(filepath):
            if fix_line_endings(filepath):
                converted_count += 1
        else:
            rospy.logwarn(f"[fix_line_endings] Arquivo não encontrado: {filepath}")
    
    if converted_count > 0:
        rospy.loginfo(f"[fix_line_endings] Conversão concluída! {converted_count} arquivo(s) convertido(s).")
    else:
        rospy.loginfo("[fix_line_endings] Nenhum arquivo precisou ser convertido (já estão com formato correto).")
    
    # Encerra o nó após concluir
    rospy.signal_shutdown("Conversão concluída")

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

