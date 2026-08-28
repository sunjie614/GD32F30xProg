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

$forbidden = 'v(add|sub|mul|div|sqrt|mla|mls|nmul|neg|abs)\.f32|__aeabi_(f|d)|__aeabi_.*2(f|d)'
$current = ''
$failures = @()
foreach ($line in $disassembly) {
    if ($line -match '^\s*[0-9a-f]+\s+<([^>]+)>:$') {
        $current = $Matches[1]
    }
    if ($line -match $forbidden) {
        $failures += "${current}: $line"
    }
}

if ($failures.Count -ne 0) {
    $failures | ForEach-Object { Write-Error $_ }
    exit 1
}

Write-Host "PASS: fixed algorithm functions contain no floating-point arithmetic instructions/calls."
