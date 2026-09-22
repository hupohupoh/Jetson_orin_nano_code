param(
    [string]$SshHost = "connect.westd.seetacloud.com",
    [int]$Port = 26365,
    [string]$User = "root",
    [Parameter(Mandatory = $true)]
    [string]$Password,
    [Parameter(Mandatory = $true)]
    [string]$NgcApiKey
)

$ErrorActionPreference = "Stop"
Import-Module Posh-SSH -ErrorAction Stop

function Run-Remote {
    param(
        [int]$SessionId,
        [string]$Command,
        [int]$Timeout = 300
    )
    $r = Invoke-SSHCommand -SessionId $SessionId -TimeOut $Timeout -Command $Command
    if ($r.ExitStatus -ne 0) {
        Write-Host $r.Output
        throw "Remote command failed: $Command"
    }
    if ($r.Output) {
        $r.Output
    }
}

$sec = ConvertTo-SecureString $Password -AsPlainText -Force
$cred = New-Object System.Management.Automation.PSCredential($User, $sec)
$session = New-SSHSession -ComputerName $SshHost -Port $Port -Credential $cred -AcceptKey
$id = $session.SessionId

try {
    Write-Host "[1/7] Check GPU"
    Run-Remote -SessionId $id -Timeout 120 -Command "nvidia-smi --query-gpu=name,driver_version,memory.total --format=csv,noheader"

    Write-Host "[2/7] Ensure Docker packages"
    Run-Remote -SessionId $id -Timeout 900 -Command "apt-get update >/tmp/apt_update.log 2>&1; apt-get install -y docker.io curl gnupg2 ca-certificates >/tmp/apt_install.log 2>&1; tail -n 8 /tmp/apt_install.log"

    Write-Host "[3/7] Ensure NVIDIA container toolkit"
    $toolkitCmd = @'
set -e
pkill -9 apt-get || true
pkill -9 dpkg || true
rm -f /var/lib/dpkg/lock-frontend /var/lib/dpkg/lock

if ! dpkg -l | grep -q nvidia-container-toolkit; then
  distribution=$(. /etc/os-release; echo $ID$VERSION_ID)
  curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey | gpg --dearmor -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg
  curl -s -L https://nvidia.github.io/libnvidia-container/$distribution/libnvidia-container.list | sed "s#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g" > /etc/apt/sources.list.d/nvidia-container-toolkit.list
  apt-get update >/tmp/nct_update.log 2>&1
  apt-get install -y nvidia-container-toolkit >/tmp/nct_install.log 2>&1
fi

dpkg -l | grep nvidia-container-toolkit
'@
    Run-Remote -SessionId $id -Timeout 1200 -Command $toolkitCmd

    Write-Host "[4/7] Configure and restart dockerd"
    $dockerCmd = @'
set -e
nvidia-ctk runtime configure --runtime=docker >/tmp/nct_config.log 2>&1 || true
pkill -9 dockerd || true
rm -f /var/run/docker.pid /var/run/docker.sock
nohup dockerd --iptables=false --bridge=none --storage-driver=vfs >/tmp/dockerd.log 2>&1 &
sleep 8
docker version --format "Server={{.Server.Version}}"
'@
    Run-Remote -SessionId $id -Timeout 240 -Command $dockerCmd

    Write-Host "[5/7] Login and pull Isaac image"
    $apiB64 = [Convert]::ToBase64String([Text.Encoding]::UTF8.GetBytes($NgcApiKey))
    $pullCmd = @"
set -e
echo $apiB64 | base64 -d | docker login nvcr.io -u '`$oauthtoken' --password-stdin
ok=0
for i in 1 2 3; do
  docker pull nvcr.io/nvidia/isaac-sim:4.5.0 && ok=1 && break
  sleep 10
done
if [ \"`$ok\" != \"1\" ]; then
  exit 1
fi
docker images | grep isaac-sim | head -n 1
"@
    Run-Remote -SessionId $id -Timeout 1800 -Command $pullCmd

    Write-Host "[6/7] Run 120s headless smoke test"
    $runCmd = @'
set -e
mkdir -p /root/isaac/cache/kit /root/isaac/cache/ov /root/isaac/cache/pip /root/isaac/data /root/isaac/logs
docker rm -f isaac-test >/dev/null 2>&1 || true
nohup docker run --name isaac-test --gpus all --network host \
  -e ACCEPT_EULA=Y -e PRIVACY_CONSENT=Y -e OMNI_KIT_ACCEPT_EULA=YES \
  -v /root/isaac/cache/kit:/isaac-sim/kit/cache:rw \
  -v /root/isaac/cache/ov:/root/.cache/ov:rw \
  -v /root/isaac/cache/pip:/root/.cache/pip:rw \
  -v /root/isaac/data:/isaac-sim/data:rw \
  nvcr.io/nvidia/isaac-sim:4.5.0 \
  /bin/bash -lc "./runheadless.sh --/app/quitAfter=120" \
  >/root/isaac/logs/isaac_test.log 2>&1 &
sleep 25
docker ps --format "table {{.Names}}\t{{.Status}}\t{{.Image}}" | sed -n '1,6p'
'@
    Run-Remote -SessionId $id -Timeout 300 -Command $runCmd

    Write-Host "[7/7] Collect smoke test log"
    $collectCmd = @'
set -e
sleep 125
docker ps -a --format "{{.Names}} {{.Status}}" | grep isaac-test || true
echo ---ISAAC_LOG_TAIL---
tail -n 80 /root/isaac/logs/isaac_test.log || true
'@
    Run-Remote -SessionId $id -Timeout 300 -Command $collectCmd

    Write-Host "Done"
}
finally {
    Remove-SSHSession -SessionId $id | Out-Null
}
