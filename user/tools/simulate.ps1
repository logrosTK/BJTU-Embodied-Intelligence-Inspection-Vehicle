<#
Run from any directory: & <workspace>/user/tools/simulate.ps1
Compiles the firmware parser/planner/output mapper on the PC and writes a 14-second CSV replay.
No hardware, ROS, Python packages, serial port or network required; GCC required.
CSV includes command references, PWM widths and CAN bytes (first byte printed first).
Motor feedback/parameters and installed signs are mock fixtures, NOT hardware measurements.
#>
$ErrorActionPreference = 'Stop'
$workspace = [System.IO.Path]::GetFullPath((Join-Path $PSScriptRoot '../..'))
$outputDirectory = Join-Path $workspace 'build/simulation'
New-Item -ItemType Directory -Path $outputDirectory -Force | Out-Null
$executable = Join-Path $outputDirectory 'control-replay.exe'
$csv = Join-Path $outputDirectory 'continuous-control.csv'
$sources = @('host_protocol.c','swerve_kinematics.c','motion_limits.c','chassis_planner.c','chassis_output.c','dm3519.c','servo_calibration.c') |
    ForEach-Object { Join-Path $workspace ('user/' + $_) }
$sources += Join-Path $PSScriptRoot 'control_replay.c'
& gcc -std=c11 -Wall -Wextra -Werror -O2 ('-I' + (Join-Path $workspace 'user')) @sources -lm -o $executable
if ($LASTEXITCODE -ne 0) { throw 'Replay build failed' }
# PS native stderr behavior differs by version; capture it without converting
# a successful simulator summary into a terminating PowerShell error.
$report = Join-Path $outputDirectory 'replay-summary.txt'
$process = Start-Process -FilePath $executable -Wait -PassThru -WindowStyle Hidden -RedirectStandardOutput $csv -RedirectStandardError $report
if ($process.ExitCode -ne 0) { throw ('Replay failed; see ' + $report) }
Get-Content -LiteralPath $report
Write-Output ('CSV: ' + $csv)
