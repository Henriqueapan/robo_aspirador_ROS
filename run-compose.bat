@echo off
REM Script para Windows 11 - Inicializa Docker Compose com suporte a GPU
REM Requisitos: Docker Desktop, WSL2, X Server (VcXsrv/Xming)

echo Verificando Docker...
docker info >nul 2>&1
if errorlevel 1 (
    echo ERRO: Docker nao esta rodando. Inicie o Docker Desktop primeiro.
    exit /b 1
)

echo Configurando DISPLAY...
REM Tenta usar WSL2 IP, se falhar usa localhost
for /f "tokens=*" %%i in ('wsl hostname -I 2^>nul') do set WSL_IP=%%i
if defined WSL_IP (
    set DISPLAY=%WSL_IP%:0.0
    echo DISPLAY configurado para WSL2: %DISPLAY%
) else (
    set DISPLAY=localhost:0.0
    echo DISPLAY configurado para localhost: %DISPLAY%
    echo Certifique-se de que VcXsrv ou Xming esta rodando!
)

echo Iniciando containers Docker...
docker-compose -f docker-compose.windows.yml up -d

if errorlevel 1 (
    echo ERRO ao iniciar containers. Verifique os logs.
    exit /b 1
) else (
    echo Containers iniciados com sucesso!
    echo Para acessar o container, use: docker exec -it ros_gazebo bash
)

