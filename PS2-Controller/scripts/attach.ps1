# attach-tl866.ps1

# Find TL866 device
Write-Host "Available USB Devices:" -ForegroundColor Cyan
usbipd list

# Prompt user for Bus ID
$busId = Read-Host "Enter the Bus ID of the TL866 (e.g., 1-7)"
usbipd bind --busid $busId
do {
    Start-Sleep -Milliseconds 300
    $status = usbipd list | Select-String $busId | Select-String "Shared"
} until ($status)
usbipd attach --wsl --busid $busId 

if ($LASTEXITCODE -eq 0) {
    Write-Host "✅ TL866 attached successfully to WSL2" -ForegroundColor Green

    # Optional: Start Docker container or build environment
    Write-Host "Launching Docker container..." -ForegroundColor Cyan
    docker compose up -d
} else {
    Write-Host "❌ Failed to attach device." -ForegroundColor Red
}