# Configuração para Windows 11

Este guia explica como configurar o ambiente ROS/Gazebo com Docker no Windows 11, incluindo suporte a GPU.

## Pré-requisitos

### 1. Instalar Docker Desktop
- Baixe e instale o [Docker Desktop para Windows](https://www.docker.com/products/docker-desktop/)
- Durante a instalação, certifique-se de habilitar o WSL2 backend

### 2. Configurar WSL2
- Instale o WSL2 com Ubuntu:
  ```powershell
  wsl --install -d Ubuntu
  ```
- Reinicie o computador se necessário

### 3. Configurar GPU NVIDIA (se aplicável)

#### Opção A: Docker Desktop com GPU Support
1. Abra Docker Desktop
2. Vá em **Settings > Resources > WSL Integration**
3. Habilite a integração com sua distribuição WSL2
4. Se você tem GPU NVIDIA, instale o [NVIDIA Container Toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html#docker-desktop)

#### Opção B: Via WSL2 diretamente
Se preferir usar Docker dentro do WSL2:
```bash
# Dentro do WSL2 (Ubuntu)
curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey | sudo gpg --dearmor -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg
curl -s -L https://nvidia.github.io/libnvidia-container/stable/deb/nvidia-container-toolkit.list | \
  sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' | \
  sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list
sudo apt-get update
sudo apt-get install -y nvidia-container-toolkit
sudo nvidia-ctk runtime configure --runtime=docker
sudo systemctl restart docker
```

### 4. Instalar X Server para GUI (VcXsrv ou Xming)

#### Opção A: VcXsrv (Recomendado)
1. Baixe e instale [VcXsrv](https://sourceforge.net/projects/vcxsrv/)
2. Inicie o XLaunch
3. Configure:
   - **Display settings**: Multiple windows
   - **Client startup**: Deixe em branco
   - **Extra settings**: Marque "Disable access control" (importante!)
4. Salve a configuração e inicie

#### Opção B: Xming
1. Baixe e instale [Xming](https://sourceforge.net/projects/xming/)
2. Inicie o Xming Server
3. Configure para permitir conexões externas

## Uso

### Método 1: PowerShell Script (Recomendado)
```powershell
# Execute no PowerShell (como Administrador se necessário)
.\run-compose.ps1
```

### Método 2: Docker Compose Manual
```powershell
# Configure o DISPLAY (ajuste conforme seu setup)
$env:DISPLAY = "localhost:0.0"  # Para VcXsrv/Xming
# OU
$env:DISPLAY = "$(wsl hostname -I).Trim():0.0"  # Para WSL2

# Execute o docker-compose
docker-compose -f docker-compose.windows.yml up -d
```

### Método 3: Via WSL2 (Mais próximo do Linux)
Se você preferir trabalhar dentro do WSL2:
```bash
# Dentro do WSL2
cd /mnt/c/caminho/para/seu/projeto
xhost +local:docker
docker-compose up -d
```

## Acessar o Container

```powershell
docker exec -it ros_gazebo bash
```

## Troubleshooting

### Problema: GUI não aparece
- Verifique se o X server (VcXsrv/Xming) está rodando
- Confirme que o DISPLAY está configurado corretamente
- No VcXsrv, certifique-se de que "Disable access control" está marcado

### Problema: GPU não funciona
- Verifique se os drivers NVIDIA estão instalados no Windows
- Confirme que o Docker Desktop está usando WSL2 backend
- Teste com: `docker run --rm --gpus all nvidia/cuda:11.0-base nvidia-smi`

### Problema: Network mode host não funciona
- No Windows, `network_mode: host` não funciona
- Use o arquivo `docker-compose.windows.yml` que mapeia as portas manualmente

## Notas Importantes

1. **Performance**: O WSL2 tem overhead de I/O. Para melhor performance, mantenha os arquivos do projeto dentro do filesystem do WSL2 (`\\wsl$\Ubuntu\home\...`)

2. **X11**: O X server precisa estar rodando antes de iniciar os containers

3. **GPU**: O suporte a GPU no Windows via Docker ainda está em desenvolvimento. Se tiver problemas, considere usar Linux nativo ou WSL2 diretamente.

