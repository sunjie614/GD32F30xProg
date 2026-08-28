param(
    [ValidateSet('IQMATH', 'FLOAT_REF')]
    [string]$Backend = 'IQMATH',
    [string]$BuildDirectory = '',
    [ValidateSet('Debug', 'Release')]
    [string]$BuildType = 'Debug'
)

$ErrorActionPreference = 'Stop'
$workspace = Split-Path -Parent $PSScriptRoot
if ([string]::IsNullOrWhiteSpace($BuildDirectory)) {
    $BuildDirectory = if ($Backend -eq 'IQMATH') {
        'build-iqmath-softfp'
    } else {
        'build-float-ref'
    }
}
$build = Join-Path $workspace $BuildDirectory

& cmake -S $workspace -B $build -GNinja `
    "-DNUMERIC_BACKEND=$Backend" "-DCMAKE_BUILD_TYPE=$BuildType"
if ($LASTEXITCODE -ne 0) {
    throw 'CMake configure failed.'
}

$database = Join-Path $build 'compile_commands.json'
$entries = Get-Content -Raw -LiteralPath $database | ConvertFrom-Json
$index = 0
foreach ($entry in $entries) {
    $index++
    Write-Host "Compile $index/$($entries.Count): $([IO.Path]::GetFileName($entry.file))"
    Push-Location $entry.directory
    cmd.exe /d /s /c $entry.command
    $code = $LASTEXITCODE
    Pop-Location
    if ($code -ne 0) {
        throw "Compile failed: $($entry.file)"
    }
}

$commands = & ninja -C $build -t commands
if ($LASTEXITCODE -ne 0) {
    throw 'Unable to obtain link commands from Ninja.'
}
$linkCommands = $commands | Where-Object {
    $_ -match 'cmake\.exe -E rm -f' -or
    $_ -match ' -o [^ ]+\.elf '
}
Push-Location $build
$index = 0
foreach ($command in $linkCommands) {
    $index++
    Write-Host "Link $index/$($linkCommands.Count)"
    cmd.exe /d /s /c $command
    if ($LASTEXITCODE -ne 0) {
        $code = $LASTEXITCODE
        Pop-Location
        throw "Archive/link step failed with exit code $code."
    }
}
Pop-Location

if ($Backend -eq 'IQMATH') {
    $algorithmObjects = @(
        'Application/CMakeFiles/User.dir/Src/foc_iq.c.obj',
        'Application/CMakeFiles/User.dir/Src/identification_iq.c.obj',
        'Application/CMakeFiles/User.dir/Src/mtpa_iq.c.obj',
        'Application/CMakeFiles/User.dir/Src/motor_iq_core.c.obj',
        'Application/CMakeFiles/User.dir/Src/protect_iq_core.c.obj',
        'Utils/CMakeFiles/Utils.dir/Src/filter_iq.c.obj',
        'Utils/CMakeFiles/Utils.dir/Src/pid_iq.c.obj',
        'Utils/CMakeFiles/Utils.dir/Src/pll_iq.c.obj',
        'Utils/CMakeFiles/Utils.dir/Src/signal_iq.c.obj',
        'Utils/CMakeFiles/Utils.dir/Src/transformation_iq.c.obj',
        'Sensorless/CMakeFiles/Sensorless.dir/Src/leso_iq.c.obj',
        'Sensorless/CMakeFiles/Sensorless.dir/Src/hf_injection_iq.c.obj',
        'Sensorless/CMakeFiles/Sensorless.dir/Src/flying_iq.c.obj'
    )
    foreach ($relativeObject in $algorithmObjects) {
        $object = Join-Path $build $relativeObject
        if (-not (Test-Path -LiteralPath $object)) {
            throw "Fixed-point audit object is missing: $relativeObject"
        }
        & (Join-Path $PSScriptRoot 'CheckFixedPointObjects.ps1') -ObjectFile $object
        if ($LASTEXITCODE -ne 0) {
            throw "Fixed-point object audit failed: $relativeObject"
        }
    }
}

Write-Host "PASS: $Backend firmware built in $build"
