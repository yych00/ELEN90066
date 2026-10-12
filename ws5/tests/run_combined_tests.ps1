# 用课程原始头文件编译控制器和离线测试，不连接机器人。
param(
    [string]$Compiler = 'gcc',
    [string]$StatechartInclude = '',
    [string]$SensorInclude = ''
)
$ErrorActionPreference = 'Stop'
$courseSource = Join-Path $PSScriptRoot '..\..\ws6\ELEN90066_2021_SM1 Courseware v1.1\ELEN90066_2021_SM1 Courseware v1.1\src'
if (!$StatechartInclude) { $StatechartInclude = Join-Path $courseSource 'C Statechart' }
if (!$SensorInclude) { $SensorInclude = Join-Path $courseSource 'Kobuki' }
$StatechartInclude = (Resolve-Path -LiteralPath $StatechartInclude).Path
$SensorInclude = (Resolve-Path -LiteralPath $SensorInclude).Path
$controllerSource = Join-Path $PSScriptRoot '..\KobukiNavigationStateChart_combined.c'
$testSource = Join-Path $PSScriptRoot 'test_combined_controller.c'
$testExecutable = Join-Path ([System.IO.Path]::GetTempPath()) ('kobuki-combined-' + [guid]::NewGuid().ToString('N') + '.exe')
$includes = @('-I', $StatechartInclude, '-I', $SensorInclude)
$scenarios = @(
    'route_stop', 'no_flat_top', 'vote_tolerance', 'bumper_on_slope',
    'cross_slope', 'invalid_acceleration', 'y_votes', 'pause_votes',
    'finish_pause', 'finish_obstacle', 'finish_cliff', 'persistent_cliff',
    'cliff_interrupts_obstacle', 'resume_bumper', 'persistent_bumper',
    'obstacle_left', 'obstacle_right', 'obstacle_center', 'overshoot',
    'angle_wrap', 'ramp_during_clear', 'finished_latch', 'reset_held_button', 'speed_cap'
)
try {
    foreach ($standard in @('gnu89', 'c99')) {
        & $Compiler "-std=$standard" -Wall -Wextra -Werror -fsyntax-only @includes $controllerSource
        if ($LASTEXITCODE -ne 0) { throw "$standard compilation failed" }
    }
    & $Compiler -std=gnu89 -Wall -Wextra -Werror -O2 @includes $testSource -o $testExecutable -lm
    if ($LASTEXITCODE -ne 0) { throw 'Test compilation failed' }
    # 分别传入真实接口和仿真接口标志；两者都只是离线输入测试。
    foreach ($mode in @('hardware', 'simulator')) {
        foreach ($scenario in $scenarios) {
            & $testExecutable $scenario $mode
            if ($LASTEXITCODE -ne 0) { throw "Failed: $scenario ($mode)" }
        }
    }
    Write-Output "PASS: $($scenarios.Count) scenarios x 2 interface modes; GNU89/C99 checks passed."
}
finally {
    if (Test-Path -LiteralPath $testExecutable) { Remove-Item -LiteralPath $testExecutable }
}
