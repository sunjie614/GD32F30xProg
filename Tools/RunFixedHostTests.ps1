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
        'Sensorless/Src/sensorless_iq.c'
    )

    $motorGatewaySources = @('Tests/motor_gateway_compat_host_test.c',
        'Application/Src/motor_iq_gateway.c', 'Application/Src/motor_iq_core.c',
        'Utils/Src/mc_math.c', 'Utils/Src/transformation_iq.c')
    Invoke-HostTest -Name 'motor_gateway_compat_q24_host_test' `
        -Definitions @('MC_NUMERIC_IQMATH=1', 'GLOBAL_Q=24', 'MATH_TYPE=0') `
        -Includes @('Tests/iqmath_mock', 'Utils/Inc', 'Application/Inc') `
        -Sources $motorGatewaySources
    Invoke-HostTest -Name 'motor_gateway_compat_float_host_test' `
        -Definitions @('MC_NUMERIC_FLOAT_REF=1') `
        -Includes @('Utils/Inc', 'Application/Inc') -Sources $motorGatewaySources
    Invoke-HostTest -Name 'protect_gateway_compat_host_test' `
        -Definitions @('MC_NUMERIC_IQMATH=1', 'GLOBAL_Q=24', 'MATH_TYPE=0') `
        -Includes @('Tests/iqmath_mock', 'Utils/Inc', 'Application/Inc',
                    'Drivers/CMSIS/CMSIS-DSP/Include', 'Drivers/CMSIS/Core/Include') `
        -Sources @('Tests/protect_gateway_compat_host_test.c',
            'Application/Src/protect_iq_core.c',
            'Utils/Src/mc_math.c')
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
                    'Application/Src/fixed_control_gateway.c',
                    'Application/Src/sensorless_iq_gateway.c',
                    'Application/Src/main_int_iq_mode.c',
                    'Application/Src/fixed_debug_gateway.c') + $focSources)

    Invoke-HostTest -Name 'sensorless_gateway_compat_host_test' `
        -Definitions @('MC_NUMERIC_IQMATH=1', 'GLOBAL_Q=24', 'MATH_TYPE=0') `
        -Includes @('Tests/iqmath_mock', 'Utils/Inc', 'Application/Inc', 'Sensorless/Inc') `
        -Sources (@('Tests/sensorless_gateway_compat_host_test.c',
                    'Application/Src/sensorless_iq_gateway.c') + $focSources)

    Invoke-HostTest -Name 'sensorless_legacy_parity_host_test' `
        -Definitions @('MC_NUMERIC_IQMATH=1', 'GLOBAL_Q=24', 'MATH_TYPE=0') `
        -Includes @('Tests/iqmath_mock', 'Utils/Inc', 'Application/Inc', 'Sensorless/Inc',
                    'Middleware/Sensorless_Interface/Inc',
                    'Drivers/CMSIS/CMSIS-DSP/Include', 'Drivers/CMSIS/Core/Include') `
        -Sources @('Tests/sensorless_legacy_parity_host_test.c',
            'Utils/Src/mc_math.c', 'Utils/Src/filter_iq.c', 'Utils/Src/pid_iq.c',
            'Utils/Src/pll_iq.c', 'Utils/Src/transformation_iq.c',
            'Utils/Src/filter.c', 'Utils/Src/pid.c',
            'Sensorless/Src/leso_iq.c', 'Sensorless/Src/hf_injection_iq.c',
            'Sensorless/Src/flying_iq.c', 'Sensorless/Src/sensorless_iq.c',
            'Drivers/CMSIS/CMSIS-DSP/Source/FastMathFunctions/arm_sin_f32.c',
            'Drivers/CMSIS/CMSIS-DSP/Source/FastMathFunctions/arm_cos_f32.c',
            'Drivers/CMSIS/CMSIS-DSP/Source/CommonTables/arm_common_tables.c')

    Invoke-HostTest -Name 'mtpa_iq_lookup_parity_host_test' `
        -Definitions @('MC_NUMERIC_IQMATH=1', 'GLOBAL_Q=24', 'MATH_TYPE=0') `
        -Includes @('Tests/iqmath_mock', 'Utils/Inc', 'Application/Inc',
                    'Drivers/CMSIS/CMSIS-DSP/Include', 'Drivers/CMSIS/Core/Include') `
        -Sources @('Tests/mtpa_iq_lookup_parity_host_test.c',
                   'Utils/Src/mc_math.c', 'Application/Src/mtpa_iq.c')

    Invoke-HostTest -Name 'fixed_debug_gateway_host_test' `
        -Definitions @('MC_NUMERIC_IQMATH=1', 'GLOBAL_Q=24', 'MATH_TYPE=0') `
        -Includes @('Tests/iqmath_mock', 'Utils/Inc', 'Application/Inc',
                    'Sensorless/Inc') `
        -Sources @('Tests/fixed_debug_gateway_host_test.c',
                   'Application/Src/fixed_debug_gateway.c',
                   'Utils/Src/mc_math.c', 'Utils/Src/transformation_iq.c')

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
    Invoke-HostTest -Name 'identification_fit_numeric_host_test' `
        -Definitions @('MC_NUMERIC_IQMATH=1', 'GLOBAL_Q=24', 'MATH_TYPE=0') `
        -Includes @('Tests/iqmath_mock', 'Utils/Inc', 'Application/Inc') `
        -Sources @('Tests/identification_fit_numeric_host_test.c',
            'Utils/Src/mc_math.c', 'Utils/Src/transformation_iq.c',
            'Application/Src/mtpa_iq.c')
    Invoke-HostTest -Name 'identification_float_host_test' `
        -Definitions @('MC_NUMERIC_FLOAT_REF=1') `
        -Includes @('Utils/Inc', 'Application/Inc') `
        -Sources $identificationSources
    Invoke-HostTest -Name 'identification_q24_host_test' `
        -Definitions @('MC_NUMERIC_IQMATH=1', 'GLOBAL_Q=24', 'MATH_TYPE=0') `
        -Includes @('Tests/iqmath_mock', 'Utils/Inc', 'Application/Inc') `
        -Sources $identificationSources

    $hardwareStopSources = @('Tests/hardware_stop_recovery_host_test.c',
        'Middleware/Hardware_Interface/Src/hardware_interface.c')
    $hardwareStopIncludes = @('Tests/hardware_interface_mock', 'Core/Inc',
        'Utils/Inc', 'Application/Inc', 'Middleware/Hardware_Interface/Inc',
        'Middleware/Communication_Interface/Inc',
        'Drivers/CMSIS/CMSIS-DSP/Include', 'Drivers/CMSIS/Core/Include')
    Invoke-HostTest -Name 'hardware_stop_recovery_iqmath_host_test' `
        -Definitions @('MC_NUMERIC_IQMATH=1') `
        -Includes $hardwareStopIncludes -Sources $hardwareStopSources
    Invoke-HostTest -Name 'hardware_stop_recovery_float_host_test' `
        -Definitions @('MC_NUMERIC_FLOAT_REF=1') `
        -Includes $hardwareStopIncludes -Sources $hardwareStopSources

    Invoke-HostTest -Name 'main_int_iq_host_test' `
        -Definitions @('MC_NUMERIC_IQMATH=1', 'GLOBAL_Q=24', 'MATH_TYPE=0') `
        -Includes @('Tests/main_int_iq_mock', 'Tests/iqmath_mock', 'Utils/Inc',
            'Application/Inc', 'Sensorless/Inc', 'Core/Inc',
            'Middleware/Hardware_Interface/Inc',
            'Middleware/Communication_Interface/Inc') `
        -Sources @('Tests/main_int_iq_host_test.c',
            'Application/Src/main_int_iq_mode.c', 'Utils/Src/telemetry_monitor.c')
}
finally {
    Pop-Location
}

Write-Host 'PASS: all fixed-point host tests completed.'
