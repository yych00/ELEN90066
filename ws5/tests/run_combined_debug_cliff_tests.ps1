# 专项离线回归，不连接机器人；两种接口参数都使用本地传感器序列。
param([string]$Compiler = 'gcc')
$ErrorActionPreference = 'Stop'
$courseSource = Join-Path $PSScriptRoot '..\..\ws6\ELEN90066_2021_SM1 Courseware v1.1\ELEN90066_2021_SM1 Courseware v1.1\src'
$includes = @('-I', (Join-Path $courseSource 'C Statechart'),
    '-I', (Join-Path $courseSource 'Kobuki'))
$controller = Join-Path $PSScriptRoot '..\KobukiNavigationStateChart_combined_debug.c'
$testSource = Join-Path $PSScriptRoot 'test_combined_debug_cliff_votes.c'
$testExecutable = Join-Path ([System.IO.Path]::GetTempPath()) ('kobuki-cliff-' + [guid]::NewGuid().ToString('N') + '.exe')
$scenarios = @('cliff_downhill_votes', 'cliff_landing_votes', 'cliff_repeated_votes',
    'cliff_ground_votes_reset', 'cliff_y_window_reset', 'cliff_pause_votes_reset', 'cliff_rolling_votes')
try {
    foreach ($standard in @('gnu89', 'c99')) {
        & $Compiler "-std=$standard" -Wall -Wextra -Werror -fsyntax-only @includes $controller
        if ($LASTEXITCODE -ne 0) { throw "$standard compilation failed" }
    }
    & $Compiler -std=gnu89 -Wall -Wextra -Werror -O2 @includes $testSource -o $testExecutable -lm
    if ($LASTEXITCODE -ne 0) { throw 'Cliff test compilation failed' }
    foreach ($mode in @('hardware', 'simulator')) {
        foreach ($scenario in $scenarios) {
            & $testExecutable $scenario $mode
            if ($LASTEXITCODE -ne 0) { throw "Failed: $scenario ($mode)" }
        }
    }
    Write-Output "PASS: $($scenarios.Count) cliff scenarios x 2 interface modes; GNU89/C99 checks passed."
}
finally {
    if (Test-Path -LiteralPath $testExecutable) { Remove-Item -LiteralPath $testExecutable }
}
