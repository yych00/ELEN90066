# 对照编译参考控制器和debug，只运行离线序列，不连接机器人。
param(
    [string]$Compiler = 'gcc',
    [string]$CourseSource = (Join-Path $PSScriptRoot '..\..\ws6\ELEN90066_2021_SM1 Courseware v1.1\ELEN90066_2021_SM1 Courseware v1.1\src')
)
$ErrorActionPreference = 'Stop'
$includes = @('-I', (Join-Path $CourseSource 'C Statechart'), '-I', (Join-Path $CourseSource 'Kobuki'))
$controller = Join-Path $PSScriptRoot '..\KobukiNavigationStateChart_combined_debug.c'
$reference = Join-Path $PSScriptRoot '..\KobukiNavigationStateChart_cliff_pitch.c'
$testSource = Join-Path $PSScriptRoot 'test_debug_pitch_reference.c'
$stem = Join-Path ([IO.Path]::GetTempPath()) ('kobuki-reference-' + [guid]::NewGuid().ToString('N'))
$referenceObject = "$stem-reference.o"
$debugObject = "$stem-debug.o"
$testExecutable = "$stem.exe"
$scenarios = @('route_y','scaled_units','speed_cap','no_stop_before_descent',
    'downhill_flag_and_stop','landing_boundary','cliff_parity','pause_y',
    'bumper_extension','flag_survives_pause_cliff')
try {
    foreach ($source in @($controller, $reference)) {
        foreach ($standard in @('gnu89','c99')) {
            & $Compiler "-std=$standard" -Wall -Wextra -Werror -fsyntax-only @includes $source
            if ($LASTEXITCODE -ne 0) { throw "Compilation failed: $source ($standard)" }
        }
    }
    & $Compiler -std=gnu89 -Wall -Wextra -Werror -O2 @includes -DKobukiNavigationStatechart=ReferenceController -c $reference -o $referenceObject
    if ($LASTEXITCODE -ne 0) { throw 'Reference compilation failed' }
    & $Compiler -std=gnu89 -Wall -Wextra -Werror -O2 @includes -DKobukiNavigationStatechart=DebugController -c $controller -o $debugObject
    if ($LASTEXITCODE -ne 0) { throw 'Debug compilation failed' }
    & $Compiler -std=gnu89 -Wall -Wextra -Werror -O2 @includes $testSource $referenceObject $debugObject -o $testExecutable -lm
    if ($LASTEXITCODE -ne 0) { throw 'Test compilation failed' }
    foreach ($mode in @('hardware','simulator')) {
        foreach ($scenario in $scenarios) {
            & $testExecutable $scenario $mode
            if ($LASTEXITCODE -ne 0) { throw "Failed: $scenario ($mode)" }
        }
    }
    Write-Output "PASS: $($scenarios.Count) scenarios x 2 interface modes; GNU89/C99 passed."
}
finally {
    foreach ($artifact in @($referenceObject,$debugObject,$testExecutable)) {
        if (Test-Path -LiteralPath $artifact) { Remove-Item -LiteralPath $artifact }
    }
}
