param([string]$Port = 'COM10')

$encoderSerial = [System.IO.Ports.SerialPort]::new(
    $Port, 115200, [System.IO.Ports.Parity]::None, 8,
    [System.IO.Ports.StopBits]::One)
$encoderSerial.WriteTimeout = 1000
try {
    $encoderSerial.Open()
    $encoderSerial.DiscardInBuffer()
    Write-Host "Live encoder output on $Port. Turn wheels by hand. Ctrl+C stops and releases the port."
    $encoderSerial.Write("`rencoder watch`r")
    while ($true) {
        $encoderText = $encoderSerial.ReadExisting()
        if ($encoderText) { Write-Host -NoNewline $encoderText }
        Start-Sleep -Milliseconds 30
    }
}
finally {
    if ($encoderSerial.IsOpen) {
        try { $encoderSerial.Write("encoder stop`r") } catch { }
        $encoderSerial.Close()
    }
    $encoderSerial.Dispose()
}
