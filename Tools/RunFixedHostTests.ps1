param(
    [string]$BuildDirectory = 'build-iqmath-softfp'
)

$ErrorActionPreference = 'Stop'
$workspace = Split-Path -Parent $PSScriptRoot
$build = Join-Path $workspace $BuildDirectory
if (-not (Test-Path -LiteralPath $build)) {
    New-Item -ItemType Directory -Path $build | Out-Null
}

function Invoke-HostTest {
    param(
        [Parameter(Mandatory = $true)]
        [string]$Name,
        [Parameter(Mandatory = $true)]
        [string[]]$Definitions,
        [Parameter(Mandatory = $true)]
        [string[]]$Includes,
        [Parameter(Mandatory = $true)]
        [string[]]$Sources
    )

    $output = Join-Path $build "$Name.exe"
    $arguments = @('-std=c11', '-O2', '-Wall', '-Wextra', '-Werror')
    $arguments += $Definitions | ForEach-Object { "-D$_" }
    $arguments += $Includes | ForEach-Object { "-I$_" }
    $arguments += $Sources
    $arguments += @('-lm', '-o', $output)

    Write-Host "Build host test: $Name"
    & gcc @arguments
    if ($LASTEXITCODE -ne 0) {
        throw "Host test compile failed: $Name"
    }
    & $output
    if ($LASTEXITCODE -ne 0) {
        throw "Host test failed: $Name"
    }
}

Push-Location $workspace
try {
    $fixedSources = @(
        'Tests/fixed_algorithm_host_test.c',
        'Utils/Src/mc_math.c',
        'Utils/Src/pid_iq.c',
        'Utils/Src/signal_iq.c',
        'Utils/Src/transformation_iq.c',
        'Application/Src/mtpa_iq.c',
        'Application/Src/motor_iq_core.c',
        'Application/Src/protect_iq_core.c'
    )
    Invoke-HostTest -Name 'fixed_algorithm_float_host_test' `
        -Definitions @('MC_NUMERIC_FLOAT_REF=1') `
        -Includes @('Utils/Inc', 'Application/Inc',
                    'Drivers/CMSIS/CMSIS-DSP/Include',
                    'Drivers/CMSIS/Core/Include') `
        -Sources $fixedSources
    Invoke-HostTest -Name 'fixed_algorithm_q24_host_test' `
        -Definitions @('MC_NUMERIC_IQMATH=1', 'GLOBAL_Q=24', 'MATH_TYPE=0') `
        -Includes @('Tests/iqmath_mock', 'Utils/Inc', 'Application/Inc') `
        -Sources $fixedSources

    Invoke-HostTest -Name 'protect_legacy_host_test' `
        -Definitions @('MC_NUMERIC_FLOAT_REF=1') `
        -Includes @('Utils/Inc', 'Application/Inc',
                    'Drivers/CMSIS/CMSIS-DSP/Include',
                    'Drivers/CMSIS/Core/Include') `
        -Sources @('Tests/protect_legacy_parity_host_test.c',
                   'Application/Src/protect.c')
    Invoke-HostTest -Name 'protect_iqmath_host_test' `
        -Definitions @('MC_NUMERIC_IQMATH=1', 'GLOBAL_Q=24', 'MATH_TYPE=0') `
        -Includes @('Tests/iqmath_mock', 'Utils/Inc', 'Application/Inc',
                    'Drivers/CMSIS/CMSIS-DSP/Include',
                    'Drivers/CMSIS/Core/Include') `
        -Sources @('Tests/protect_legacy_parity_host_test.c',
                   'Application/Src/protect_iq_gateway.c',
                   'Application/Src/protect_iq_core.c',
                   'Utils/Src/mc_math.c')

    $focSources = @(
        'Utils/Src/mc_math.c',
        'Utils/Src/filter_iq.c',
        'Utils/Src/pid_iq.c',
        'Utils/Src/pll_iq.c',
        'Utils/Src/signal_iq.c',
        'Utils/Src/transformation_iq.c',
        'Application/Src/foc_iq.c',
        'Application/Src/identification_iq.c',
        'Application/Src/mtpa_iq.c',
        'Sensorless/Src/leso_iq.c',
        'Sensorless/Src/hf_injection_iq.c',
        'Sensorless/Src/flying_iq.c'
    )
    Invoke-HostTest -Name 'foc_iq_host_test' `
        -Definitions @('MC_NUMERIC_IQMATH=1', 'GLOBAL_Q=24', 'MATH_TYPE=0') `
        -Includes @('Tests/iqmath_mock', 'Utils/Inc', 'Application/Inc',
                    'Sensorless/Inc') `
        -Sources (@('Tests/foc_iq_host_test.c') + $focSources)

    Invoke-HostTest -Name 'fixed_gateway_mode_host_test' `
        -Definitions @('MC_NUMERIC_IQMATH=1', 'GLOBAL_Q=24', 'MATH_TYPE=0') `
        -Includes @('Tests/iqmath_mock', 'Utils/Inc', 'Application/Inc',
                    'Sensorless/Inc') `
        -Sources (@('Tests/fixed_gateway_mode_host_test.c',
                    'Application/Src/fixed_control_gateway.c') + $focSources)

    Invoke-HostTest -Name 'telemetry_monitor_host_test' `
        -Definitions @('MC_NUMERIC_IQMATH=1') `
        -Includes @('Utils/Inc') `
        -Sources @('Tests/telemetry_monitor_host_test.c',
                   'Utils/Src/telemetry_monitor.c')

    $identificationSources = @(
        'Tests/identification_iq_host_test.c',
        'Utils/Src/mc_math.c',
        'Utils/Src/transformation_iq.c',
        'Application/Src/identification_iq.c',
        'Application/Src/mtpa_iq.c'
    )
    Invoke-HostTest -Name 'identification_float_host_test' `
        -Definitions @('MC_NUMERIC_FLOAT_REF=1') `
        -Includes @('Utils/Inc', 'Application/Inc') `
        -Sources $identificationSources
    Invoke-HostTest -Name 'identification_q24_host_test' `
        -Definitions @('MC_NUMERIC_IQMATH=1', 'GLOBAL_Q=24', 'MATH_TYPE=0') `
        -Includes @('Tests/iqmath_mock', 'Utils/Inc', 'Application/Inc') `
        -Sources $identificationSources
}
finally {
    Pop-Location
}

Write-Host 'PASS: all fixed-point host tests completed.'
