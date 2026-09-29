param(
    [string]$BuildDirectory = 'build-iqmath-softfp',
    [string]$PythonExecutable = ''
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

$pythonCandidates = @()
if ($PythonExecutable) { $pythonCandidates += $PythonExecutable }
if ($env:PYTHON) { $pythonCandidates += $env:PYTHON }
$bundledPython = Join-Path $env:USERPROFILE `
    '.cache/codex-runtimes/codex-primary-runtime/dependencies/python/python.exe'
if (Test-Path -LiteralPath $bundledPython) {
    $pythonCandidates += $bundledPython
}
foreach ($name in @('py.exe', 'python.exe', 'python3.exe')) {
    $candidate = Get-Command $name -ErrorAction SilentlyContinue
    if ($candidate -and $candidate.Source -notmatch '[\\/]WindowsApps[\\/]') {
        $pythonCandidates += $candidate.Source
    }
}
$python = $pythonCandidates | Select-Object -First 1
if (-not $python) {
    throw 'Python not found. Install Python or pass -PythonExecutable.'
}

& $python (Join-Path $PSScriptRoot 'A2LFilter.py') `
    --input $a2l `
    --output $a2l
if ($LASTEXITCODE -ne 0) {
    throw 'A2LFilter.py failed.'
}

Write-Host "PASS: A2L generated at $a2l"
