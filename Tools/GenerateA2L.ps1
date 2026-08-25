param(
    [string]$BuildDirectory = 'build-iqmath-softfp'
)

$ErrorActionPreference = 'Stop'
$workspace = Split-Path -Parent $PSScriptRoot
$projectName = Split-Path -Leaf $workspace
$build = Join-Path $workspace $BuildDirectory
$elf = Join-Path $build "$projectName.elf"
$a2l = Join-Path $build "$projectName.a2l"

if (-not (Test-Path -LiteralPath $elf)) {
    throw "ELF not found: $elf. Run the IQMATH build first."
}

& (Join-Path $PSScriptRoot 'a2ltool.exe') `
    --create `
    --elffile $elf `
    --measurement-regex '.*' `
    --a2lversion '1.5.0' `
    --output $a2l
if ($LASTEXITCODE -ne 0) {
    throw 'a2ltool failed.'
}

& python (Join-Path $PSScriptRoot 'A2LFilter.py') `
    --input $a2l `
    --output $a2l
if ($LASTEXITCODE -ne 0) {
    throw 'A2LFilter.py failed.'
}

Write-Host "PASS: A2L generated at $a2l"
