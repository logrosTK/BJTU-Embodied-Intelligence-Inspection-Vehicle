$ErrorActionPreference = 'Stop'
$workspace = [System.IO.Path]::GetFullPath((Join-Path $PSScriptRoot '../..'))
$testOutput = Join-Path $workspace 'build/host-tests'
New-Item -ItemType Directory -Path $testOutput -Force | Out-Null
$testExe = Join-Path $testOutput 'firmware-tests.exe'
$sourceNames = @('host_protocol.c','hwt101ct.c','dm3519.c','uart_dma.c','can_link.c','swerve_kinematics.c','motion_limits.c','chassis_planner.c','chassis_output.c','drive_supervisor.c','single_wheel_test.c','servo_calibration.c','servo_pwm.c','inspection_app.c')
$sources = @($sourceNames | ForEach-Object { Join-Path $workspace ('user/' + $_) })
$sources += Join-Path $PSScriptRoot 'test_protocols.c'
$sources += Join-Path $PSScriptRoot 'test_transport.c'
$sources += Join-Path $PSScriptRoot 'test_kinematics.c'
$sources += Join-Path $PSScriptRoot 'test_motion.c'
$sources += Join-Path $PSScriptRoot 'test_planner.c'
$sources += Join-Path $PSScriptRoot 'test_output.c'
$sources += Join-Path $PSScriptRoot 'test_supervisor.c'
& gcc -std=c11 -Wall -Wextra -Werror -O2 ('-I' + (Join-Path $PSScriptRoot 'mocks')) ('-I' + (Join-Path $workspace 'user')) @sources -lm -o $testExe
if ($LASTEXITCODE -ne 0) { throw 'Native test build failed' }
& $testExe
if ($LASTEXITCODE -ne 0) { throw 'Firmware tests failed' }
