$ErrorActionPreference='Stop'
docker compose pull
docker compose up -d
docker ps --filter name=water-treatment-openplc
Write-Host 'Runtime management endpoint: https://localhost:8443 (use OpenPLC Editor, not a browser)'
Write-Host 'Modbus TCP endpoint: 127.0.0.1:5020'
