# 编译调试工程控制器并执行路线回归测试，不连接机器人。
param([string]$Compiler = 'gcc')
$ErrorActionPreference = 'Stop'
$courseSource = Join-Path $PSScriptRoot '..\..\ws6\ELEN90066_2021_SM1 Courseware v1.1\ELEN90066_2021_SM1 Courseware v1.1\src'
$includes = @('-I', (Join-Path $courseSource 'C Statechart'),
    '-I', (Join-Path $courseSource 'Kobuki'))
$controller = Join-Path $PSScriptRoot '..\KobukiNavigationStateChart_combined_debug.c'
$testSource = Join-Path $PSScriptRoot 'test_combined_debug_controller.c'
$testExecutable = Join-Path ([System.IO.Path]::GetTempPath()) ('kobuki-debug-' + [guid]::NewGuid().ToString('N') + '.exe')
$scenarios = @(
    'route_stop', 'no_flat_top', 'vote_tolerance', 'bumper_on_slope',
    'cross_slope', 'invalid_acceleration', 'y_votes', 'pause_votes',
    'finish_pause', 'finish_obstacle', 'finish_cliff', 'persistent_cliff',
    'cliff_interrupts_obstacle', 'resume_bumper', 'persistent_bumper',
    'backup_increasing_distance', 'cliff_increasing_distance',
    'backup_start_offsets', 'cliff_start_offsets',
    'backup_distance_overflow', 'cliff_distance_overflow',
    'obstacle_left', 'obstacle_right', 'obstacle_center', 'overshoot',
    'angle_wrap', 'ramp_during_clear', 'finished_latch', 'reset_held_button', 'speed_cap',
    'approach_heading_target', 'clear_heading_hold', 'clear_distance_direction',
    'repeated_collision_heading',
    'course_20ms', 'course_64ms', 'course_100ms', 'course_250ms',
    'course_jitter', 'course_capped200', 'course_wheel_drift'
)
try {
    foreach ($standard in @('gnu89', 'c99')) {
        & $Compiler "-std=$standard" -Wall -Wextra -Werror -fsyntax-only @includes $controller
        if ($LASTEXITCODE -ne 0) { throw "$standard compilation failed" }
    }
    & $Compiler -std=gnu89 -Wall -Wextra -Werror -O2 @includes $testSource -o $testExecutable -lm
    if ($LASTEXITCODE -ne 0) { throw 'Debug test compilation failed' }
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
