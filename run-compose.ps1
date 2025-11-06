# Script para Windows 11 - Inicializa Docker Compose com suporte a GPU
# Requisitos:
# - Docker Desktop instalado e rodando
# - WSL2 configurado
# - NVIDIA GPU com drivers instalados (opcional)
# - Docker Desktop configurado para usar GPU (Settings > Resources > WSL Integration)
# - X Server rodando (VcXsrv ou Xming) para GUI

# Verifica se Docker está rodando
try {
    docker info | Out-Null
} catch {
    Write-Host "ERRO: Docker não está rodando. Inicie o Docker Desktop primeiro." -ForegroundColor Red
    exit 1
}

# Tenta detectar o IP do WSL2
try {
    $wslIp = (wsl hostname -I 2>$null).Trim()
    if ($wslIp) {
        $env:DISPLAY = "$wslIp`:0.0"
        Write-Host "DISPLAY configurado para WSL2: $env:DISPLAY" -ForegroundColor Green
    } else {
        throw
    }
} catch {
    # Fallback: usa localhost (para VcXsrv/Xming)
    $env:DISPLAY = "localhost:0.0"
    Write-Host "DISPLAY configurado para localhost: $env:DISPLAY" -ForegroundColor Yellow
    Write-Host "Certifique-se de que VcXsrv ou Xming está rodando!" -ForegroundColor Yellow
}

# Inicia os containers usando o arquivo específico para Windows
Write-Host "Iniciando containers Docker..." -ForegroundColor Green
docker-compose -f docker-compose.windows.yml up -d

if ($LASTEXITCODE -eq 0) {
    Write-Host "Containers iniciados com sucesso!" -ForegroundColor Green
    Write-Host "Para acessar o container, use: docker exec -it ros_gazebo bash" -ForegroundColor Cyan
} else {
    Write-Host "ERRO ao iniciar containers. Verifique os logs com: docker-compose -f docker-compose.windows.yml logs" -ForegroundColor Red
}

