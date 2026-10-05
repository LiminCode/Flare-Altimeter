# Launch the BNO08x live attitude viewer.
#
#   .\tools\viewer.ps1              # read the XIAO on COM7
#   .\tools\viewer.ps1 --demo       # synthetic motion, no hardware needed
#   .\tools\viewer.ps1 --port COM4  # the bench board
#   .\tools\viewer.ps1 --replay tools\mock_descent_20m.csv
#                                   # feed a mock 20 m descent to the board as
#                                   # laser frames, watch the graph, then check
#
# The serial port allows exactly one reader, so any PlatformIO monitor holding
# it is closed first -- the same collision that breaks uploads.

$ErrorActionPreference = 'Stop'

$venvPython = Join-Path $env:USERPROFILE '.venvs\flare-imu\Scripts\python.exe'
$viewer = Join-Path $PSScriptRoot 'imu_viewer.py'

if (-not (Test-Path $venvPython)) {
    Write-Error "venv missing at $venvPython. Recreate it with:
  & `"`$env:USERPROFILE\.platformio\python3\python.exe`" -m venv `"`$env:USERPROFILE\.venvs\flare-imu`"
  & `"`$env:USERPROFILE\.venvs\flare-imu\Scripts\python.exe`" -m pip install -r `"$PSScriptRoot\requirements.txt`""
}

if ($args -notcontains '--demo') {
    Get-CimInstance Win32_Process |
        Where-Object { $_.Name -in @('platformio.exe', 'python.exe') -and
                       $_.CommandLine -like '*device monitor*' } |
        ForEach-Object {
            Write-Host "closing serial monitor (PID $($_.ProcessId))" -ForegroundColor Yellow
            Stop-Process -Id $_.ProcessId -Force -ErrorAction SilentlyContinue
        }
    Start-Sleep -Milliseconds 400
}

& $venvPython $viewer @args
