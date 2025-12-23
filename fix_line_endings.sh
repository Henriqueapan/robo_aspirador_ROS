#!/bin/bash
# Script para converter terminações de linha de CRLF para LF nos arquivos Python

cd src/robo_aspirador/scripts

# Converte os arquivos Python
for file in robo_aspirador_controller.py navigation_planner.py robo_aspirador_keyteleop.py; do
    if [ -f "$file" ]; then
        echo "Convertendo $file..."
        sed -i 's/\r$//' "$file"
        # Garante que o arquivo termina com LF
        if [ -n "$(tail -c 1 "$file")" ]; then
            echo "" >> "$file"
        fi
    fi
done

echo "Conversão concluída!"

