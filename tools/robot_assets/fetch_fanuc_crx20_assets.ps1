param(
    [string]$OutDir = "frontend/assets/robot/crx20ial"
)

$ErrorActionPreference = "Stop"
$ProgressPreference = "SilentlyContinue"

function Download-File {
    param(
        [string]$Url,
        [string]$Path
    )
    New-Item -ItemType Directory -Force -Path (Split-Path -Parent $Path) | Out-Null
    Invoke-WebRequest -Uri $Url -OutFile $Path -UseBasicParsing
    Write-Host "Downloaded $Path"
}

$visualNames = @("base", "j1", "j2", "j3", "j4", "j5", "j6")
$collisionNames = @("base", "j1", "j2", "j3", "j4", "j5", "j6")

$baseMedia = "https://media.githubusercontent.com/media/FANUC-CORPORATION/fanuc_description/main/fanuc_crx_description/meshes/crx20ia_l"

foreach ($name in $visualNames) {
    $url = "$baseMedia/visual/$name.dae"
    $dst = Join-Path $OutDir "meshes/crx20ial/visual/$name.dae"
    Download-File -Url $url -Path $dst
}

foreach ($name in $collisionNames) {
    $url = "$baseMedia/collision/$name.stl"
    $dst = Join-Path $OutDir "meshes/crx20ial/collision/$name.stl"
    Download-File -Url $url -Path $dst
}

Write-Host ""
Write-Host "Next steps:"
Write-Host "1. Generate a plain URDF from fanuc_crx_description/robot/crx20ia_l.urdf.xacro (outside this script)."
Write-Host "2. Rewrite package://fanuc_crx_description/... mesh paths to /assets/robot/crx20ial/..."
Write-Host "3. Save the result as $OutDir/robot.urdf"

