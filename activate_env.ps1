# Dot-source this to use the project's .venv in the current PowerShell session:
#   . .\activate_env.ps1
# (Plain `conda activate .\.venv` won't work unless `conda init powershell` has
# been run for this profile -- this script sets PATH directly instead, which
# also picks up the conda-forge BLAS/LAPACK DLLs numpy needs at
# .venv\Library\bin.)

$envRoot = Join-Path $PSScriptRoot ".venv"

if (-not (Test-Path (Join-Path $envRoot "python.exe"))) {
    Write-Error "No .venv found at $envRoot. Run: conda create -p .\.venv python=3.12 -y"
    return
}

$env:CONDA_PREFIX = $envRoot
$env:PATH = "$envRoot\Library\bin;$envRoot;$envRoot\Scripts;$env:PATH"

Write-Host "Activated .venv ($envRoot). 'python' now points to it." -ForegroundColor Green
