param(
    [Parameter(Mandatory = $true)]
    [string]$ObjectFile
)

$ErrorActionPreference = 'Stop'
$object = Resolve-Path -LiteralPath $ObjectFile
$disassembly = & arm-none-eabi-objdump -d $object
if ($LASTEXITCODE -ne 0) {
    throw "objdump failed for $object"
}

$coreFunctions = @(
    'fixed_pi_step',
    'fixed_clarke',
    'fixed_park',
    'fixed_inverse_park',
    'fixed_svpwm',
    'fixed_observer_step',
    'fixed_mtpa_id',
    'fixed_identification_step'
)
$forbidden = 'v(add|sub|mul|div|sqrt)\.f32|__aeabi_f(add|sub|mul|div|2d|d2f)'
$current = ''
$failures = @()
foreach ($line in $disassembly) {
    if ($line -match '^\s*[0-9a-f]+\s+<([^>]+)>:$') {
        $current = $Matches[1]
    }
    if ($current -in $coreFunctions -and $line -match $forbidden) {
        $failures += "${current}: $line"
    }
}

if ($failures.Count -ne 0) {
    $failures | ForEach-Object { Write-Error $_ }
    exit 1
}

Write-Host "PASS: fixed algorithm functions contain no floating-point arithmetic instructions/calls."
