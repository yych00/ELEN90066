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
    'ramp_entry_persistent_bumper', 'ramp_entry_oblique_clear', 'ramp_entry_flat_collision_impulse',
    'ramp_entry_lateral_bumper', 'ramp_entry_pause_confirmation', 'ramp_entry_cliff_priority', 'ramp_entry_no_premature_top',
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
    'configured_course', 'y_limited_pair', 'y_sensitive_switching', 'invalid_y_input', 'finish_distance_wrap',
    'finish_distance_frozen', 'ramp_clear_vote_threshold', 'no_slope_during_turn',
    'stop_floor_bias', 'stop_shallow_descent', 'stop_noisy_level', 'stop_reject_five_level_votes',
    'stop_reject_top', 'stop_pause_keeps_descent', 'stop_reference_fixed_on_resume',
    'stop_start_reference_guard', 'stop_invalid_level', 'stop_latest_level_required', 'stop_vote_outlier_last',
    'crest_braking_before_top', 'crest_braking_after_top', 'crest_repeated_pulses',
    'descent_distance_threshold', 'descent_pause_resets_candidate', 'descent_invalid_gap', 'descent_distance_wrap',
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
    # 对不同实机接线/单位/安装角配置，分别走完上坡、坡顶、下坡和终点停车。
    $variants = @(
        @{ Name='si_units'; Flags=@('-DACCEL_UNITS_PER_G=9.80665'); Scenario='configured_course' },
        @{ Name='axis_mapping'; Flags=@('-DACCEL_FORWARD_AXIS=1','-DACCEL_SIDE_AXIS=0','-DACCEL_FORWARD_SIGN=-1','-DACCEL_SIDE_SIGN=-1','-DACCEL_VERTICAL_SIGN=-1'); Scenario='configured_course' },
        @{ Name='mounting_offsets'; Flags=@('-DACCEL_MOUNT_ROLL_DEG=3.0','-DACCEL_MOUNT_PITCH_DEG=4.0'); Scenario='configured_course' },
        @{ Name='reverse_odometry'; Flags=@('-DDISTANCE_FORWARD_SIGN=-1'); Scenario='configured_course' },
        @{ Name='invalid_units'; Flags=@('-DACCEL_UNITS_PER_G=0.0'); Scenario='bad_accel_configuration' }
    )
    foreach ($variant in $variants) {
        $flags = $variant.Flags
        & $Compiler -std=gnu89 -Wall -Wextra -Werror -O2 @includes @flags $testSource -o $testExecutable -lm
        if ($LASTEXITCODE -ne 0) { throw "Variant compilation failed: $($variant.Name)" }
        foreach ($mode in @('hardware', 'simulator')) {
            & $testExecutable $variant.Scenario $mode
            if ($LASTEXITCODE -ne 0) { throw "Variant failed: $($variant.Name) ($mode)" }
        }
        Write-Output "PASS configuration: $($variant.Name)"
    }
    Write-Output "PASS: $($scenarios.Count) scenarios x 2 interface modes + $($variants.Count) configurations x 2 modes; GNU89/C99 checks passed."
}
finally {
    if (Test-Path -LiteralPath $testExecutable) { Remove-Item -LiteralPath $testExecutable }
}
