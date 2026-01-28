#!/bin/bash
# Script para converter terminações de linha automaticamente
# Executado pelo launch file

SCRIPT_DIR="$(rospack find robo_aspirador)/scripts"
WORKSPACE_ROOT="$(rospack find robo_aspirador)/../.."

# Tenta encontrar o script de conversão
FIX_SCRIPT=""
if [ -f "$WORKSPACE_ROOT/fix_line_endings.py" ]; then
    FIX_SCRIPT="$WORKSPACE_ROOT/fix_line_endings.py"
elif [ -f "$(rospack find robo_aspirador)/../../fix_line_endings.py" ]; then
    FIX_SCRIPT="$(rospack find robo_aspirador)/../../fix_line_endings.py"
fi

if [ -n "$FIX_SCRIPT" ] && [ -f "$FIX_SCRIPT" ]; then
    echo "[fix_line_endings] Convertendo terminações de linha dos scripts Python..."
    cd "$WORKSPACE_ROOT" && python3 "$FIX_SCRIPT" 2>/dev/null || echo "[fix_line_endings] Aviso: Conversão não necessária ou já concluída"
else
    # Fallback: converte diretamente com sed
    echo "[fix_line_endings] Convertendo terminações de linha diretamente..."
    if [ -d "$SCRIPT_DIR" ]; then
        cd "$SCRIPT_DIR"
        for file in robo_aspirador_controller.py navigation_planner.py robo_aspirador_keyteleop.py; do
            if [ -f "$file" ]; then
                sed -i 's/\r$//' "$file" 2>/dev/null && echo "[fix_line_endings] Convertido: $file" || true
            fi
        done
    fi
fi

echo "[fix_line_endings] Conversão de terminações de linha concluída."

