$env:HTTP_PROXY = "http://127.0.0.1:10808"
$env:HTTPS_PROXY = "http://127.0.0.1:10808"
$env:ALL_PROXY = "http://127.0.0.1:10808"
$env:NO_PROXY = "localhost,127.0.0.1,::1"

Write-Host "Proxy enabled: http://127.0.0.1:10808"
