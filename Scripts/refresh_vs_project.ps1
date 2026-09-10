param(
    [string]$Engine = 'D:/UE5/UE_5.8'
)

$ErrorActionPreference = 'Stop'
$ProjectRoot = Split-Path -Parent $PSScriptRoot
$ProjectFile = Join-Path $ProjectRoot 'pdd_cloth.uproject'
$BuildScript = Join-Path $Engine 'Engine/Build/BatchFiles/Build.bat'
if (!(Test-Path -LiteralPath $BuildScript)) {
    throw "Unreal Build.bat not found: $BuildScript. Specify -Engine with your UE installation."
}

# Explicit format prevents the user's default generator from selecting VS Code.
& $BuildScript -projectfiles "-project=$ProjectFile" -game -rocket -2022
if ($LASTEXITCODE -ne 0) {
    throw "Visual Studio project generation failed (exit $LASTEXITCODE)."
}
Write-Host 'Visual Studio project refreshed. Reload the solution if Visual Studio prompts you.'
