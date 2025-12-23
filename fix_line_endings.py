#!/usr/bin/env python3
"""
Script para converter terminações de linha de CRLF para LF nos arquivos Python.
Execute este script dentro do container Docker ou no ambiente Linux.
"""

import os

def fix_line_endings(filepath):
    """Converte CRLF para LF em um arquivo"""
    try:
        with open(filepath, 'rb') as f:
            content = f.read()
        
        # Remove \r
        content = content.replace(b'\r\n', b'\n').replace(b'\r', b'\n')
        
        # Garante que termina com \n
        if not content.endswith(b'\n'):
            content += b'\n'
        
        with open(filepath, 'wb') as f:
            f.write(content)
        
        print(f"Convertido: {filepath}")
        return True
    except Exception as e:
        print(f"Erro ao converter {filepath}: {e}")
        return False

if __name__ == '__main__':
    import sys
    
    # Tenta encontrar o diretório do workspace
    # Primeiro tenta usar ROS_PACKAGE_PATH
    script_dir = None
    if 'ROS_PACKAGE_PATH' in os.environ:
        for path in os.environ['ROS_PACKAGE_PATH'].split(':'):
            potential_dir = os.path.join(path, 'robo_aspirador', 'scripts')
            if os.path.exists(potential_dir):
                script_dir = potential_dir
                break
    
    # Se não encontrou, tenta caminhos relativos comuns
    if script_dir is None:
        # Tenta a partir do diretório atual
        potential_paths = [
            os.path.join('src', 'robo_aspirador', 'scripts'),
            os.path.join('..', 'src', 'robo_aspirador', 'scripts'),
            os.path.join(os.path.dirname(__file__), 'src', 'robo_aspirador', 'scripts'),
            os.path.join(os.path.dirname(__file__), '..', 'src', 'robo_aspirador', 'scripts'),
        ]
        
        for potential_path in potential_paths:
            if os.path.exists(potential_path):
                script_dir = potential_path
                break
    
    if script_dir is None:
        print("Aviso: Não foi possível encontrar o diretório de scripts. Pulando conversão.")
        sys.exit(0)
    
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
            print(f"Aviso: Arquivo não encontrado: {filepath}")
    
    if converted_count > 0:
        print(f"Conversão concluída! {converted_count} arquivo(s) convertido(s).")
    else:
        print("Nenhum arquivo precisou ser convertido (já estão com formato correto).")

