# Replay mock laser data through the real firmware, with the UI showing what the
# board actually reports. This is the end-to-end test: unit tests prove the logic,
# this proves the logic is wired to the hardware.
#
#   .\tools\replay.ps1                 # every scenario, in one window
#   .\tools\replay.ps1 perfect         # just one
#   .\tools\replay.ps1 bad accel       # a few, in order
#   .\tools\replay.ps1 -Regenerate     # rebuild the mock files first
#
# Scenarios live in tools\mock\*.csv and carry their own pass/fail expectations in
# their headers, so adding one is a data change, not a code change --
# see tools\make_mock_descent.py.
#
# Exit code is 0 only if every scenario passed, so this can gate a commit.
#
# 中文：用真实固件回放模拟激光数据，UI 显示的是板子实际报出来的东西。
#       单元测试证明逻辑本身对，这个证明逻辑和硬件之间接对了。
#       每个场景的判定标准写在它自己的 CSV 头部，加场景只改数据不改代码。

param(
    [Parameter(ValueFromRemainingArguments = $true)]
    [string[]]$Scenarios,
    [switch]$Regenerate
)

$ErrorActionPreference = 'Stop'

$root = Split-Path $PSScriptRoot -Parent
$venvPython = Join-Path $env:USERPROFILE '.venvs\flare-imu\Scripts\python.exe'
$mockDir = Join-Path $PSScriptRoot 'mock'

if (-not (Test-Path $venvPython)) {
    Write-Error "venv missing at $venvPython. See tools\requirements.txt."
}

if ($Regenerate -or -not (Test-Path $mockDir)) {
    & $venvPython (Join-Path $PSScriptRoot 'make_mock_descent.py')
}

if (-not $Scenarios -or $Scenarios.Count -eq 0) {
    # Deliberate order: prove the clean case first. If 'perfect' fails there is no
    # point reading the hostile results -- something basic is broken and the
    # hostile file has too many excuses in it to tell you what.
    # 中文：顺序是故意的，先跑干净的场景。'perfect' 挂了就不必看恶劣场景的结果 ——
    #       那说明基本功能就坏了，而恶劣场景里"借口"太多，看不出真正的原因。
    $Scenarios = @('perfect', 'accel', 'bad', 'noarm')
}

# @() is not decoration. A foreach that yields ONE item produces a String, not an
# array, and splatting a String with @files passes it one character at a time --
# Python then reports 'No such file or directory: C'. Forcing an array makes the
# single-scenario case behave like the multi-scenario one.
# 中文：@() 不是装饰。foreach 只产出一个元素时得到的是字符串而不是数组，
#       而用 @files splat 一个字符串会把它**按字符**拆成一个个参数 ——
#       Python 于是报「找不到文件 C」。强制成数组，单场景才和多场景行为一致。
$files = @(foreach ($s in $Scenarios) {
    $p = if (Test-Path $s) { $s } else { Join-Path $mockDir "$s.csv" }
    if (-not (Test-Path $p)) {
        Write-Error "no such scenario: $s (looked for $p). Try -Regenerate."
    }
    $p
})

# The serial port allows exactly one reader and this script is about to need it.
# Another viewer counts: since the viewer gained --replay it is the most likely
# thing holding COM4, more likely than a leftover PlatformIO monitor.
# 中文：串口同时只允许一个读取者，而这个脚本马上就要用它。
#       另一个查看器实例也算 —— 自从查看器支持 --replay 之后，
#       占着 COM4 的最可能就是它，而不是遗留的 PlatformIO monitor。
Get-CimInstance Win32_Process |
    Where-Object { $_.Name -in @('platformio.exe', 'python.exe', 'pythonw.exe') -and
                   ($_.CommandLine -like '*device monitor*' -or
                    $_.CommandLine -like '*imu_viewer.py*') } |
    ForEach-Object {
        Write-Host "closing process holding the port (PID $($_.ProcessId))" -ForegroundColor Yellow
        Stop-Process -Id $_.ProcessId -Force -ErrorAction SilentlyContinue
    }
Start-Sleep -Milliseconds 600

Write-Host "replaying: $($Scenarios -join ', ')" -ForegroundColor Cyan
& $venvPython (Join-Path $PSScriptRoot 'imu_viewer.py') --replay @files
exit $LASTEXITCODE
