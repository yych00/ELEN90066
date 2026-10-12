# 编译调试控制器，使用 LED 写入桩测试，不连接机器人。
param([string]$Compiler = 'gcc')
$ErrorActionPreference = 'Stop'
$courseSource = Join-Path $PSScriptRoot '..\..\ws6\ELEN90066_2021_SM1 Courseware v1.1\ELEN90066_2021_SM1 Courseware v1.1\src'
$includes = @('-I', (Join-Path $courseSource 'C Statechart'),
    '-I', (Join-Path $courseSource 'Kobuki'), '-I', (Join-Path $courseSource 'myRIO'))
$controller = Join-Path $PSScriptRoot '..\KobukiNavigationStateChart_combined_debug.c'
$testSource = Join-Path $PSScriptRoot 'test_combined_debug_controller.c'
$testExecutable = Join-Path ([System.IO.Path]::GetTempPath()) ('kobuki-debug-' + [guid]::NewGuid().ToString('N') + '.exe')
$scenarios = @(
    'route_stop', 'no_flat_top', 'vote_tolerance', 'bumper_on_slope',
    'cross_slope', 'invalid_acceleration', 'y_votes', 'pause_votes',
    'finish_pause', 'finish_obstacle', 'finish_cliff', 'persistent_cliff',
    'cliff_interrupts_obstacle', 'resume_bumper', 'persistent_bumper',
    'obstacle_left', 'obstacle_right', 'obstacle_center', 'overshoot',
    'angle_wrap', 'ramp_during_clear', 'finished_latch', 'reset_held_button', 'speed_cap',
    'course_20ms', 'course_64ms', 'course_100ms', 'course_250ms',
    'course_jitter', 'course_capped200', 'course_wheel_drift'
)
try {
    foreach ($standard in @('gnu89', 'c99')) {
        # 课程 SDK 只附带 1900 的 FPGA 头文件，检查该实机配置及禁用灯控的桌面配置。
        foreach ($target in @('-DMyRio_1900', '-DDEBUG_ENABLE_LED=0')) {
            & $Compiler "-std=$standard" -Wall -Wextra -Werror -fsyntax-only $target @includes $controller
            if ($LASTEXITCODE -ne 0) { throw "$standard $target compilation failed" }
        }
    }
    & $Compiler -std=gnu89 -Wall -Wextra -Werror -O2 @includes $testSource -o $testExecutable -lm
    if ($LASTEXITCODE -ne 0) { throw 'Debug test compilation failed' }
    & $testExecutable
    if ($LASTEXITCODE -ne 0) { throw 'LED transition tests failed' }
    foreach ($mode in @('hardware', 'simulator')) {
        foreach ($scenario in $scenarios) {
            & $testExecutable $scenario $mode
            if ($LASTEXITCODE -ne 0) { throw "Failed: $scenario ($mode)" }
        }
    }
    Write-Output "PASS: LED checks + $($scenarios.Count) scenarios x 2 interface modes; GNU89/C99 checks passed."
}
finally {
    if (Test-Path -LiteralPath $testExecutable) { Remove-Item -LiteralPath $testExecutable }
}
