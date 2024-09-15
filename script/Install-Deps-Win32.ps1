$depsDir = "$PSScriptRoot\..\.deps"

$libProfile = $Host.UI.PromptForChoice("Profile", "What profile do you use?", @("&Release","&Debug"), 1)

$depsLibsDir = ""
$middleName = ""
$lastName = ""
if ($libProfile -eq 0) {
    $middleName = "-"
    $depsLibsDir = "$depsDir\release"
    $lastName = "-win32-x64"
} else {
    $middleName = "-devel-"

    $compiler = $Host.UI.PromptForChoice("Compiler", "What compiler do you use?", @("&VC", "&MingW"), 0);
    
    if ($compiler -eq 0) {
        $lastName = "-VC"
        $depsLibsDir = "$depsDir\debug-vc"
    } else {
        $lastName = "-mingw"
        $depsLibsDir = "$depsDir\debug-mingw"
    }
}

if (!(Test-Path -PathType Container $depsDir))
{
    New-Item  -ItemType Directory  -Path $depsDir | Out-Null
}

if (Test-Path -PathType Container $depsLibsDir) {
    Remove-Item -Recurse -Force $depsLibsDir
}

New-Item -ItemType Directory -Path $depsLibsDir | Out-Null 

$SDLRootUrl = "https://github.com/libsdl-org"

$SDLDepsList = @(
    @{
        Repo = "SDL"
        Version = "2.30.3"
        DownloadName = "SDL2"
    },
    @{
        Repo = "SDL_ttf"
        Version = "2.22.0"
        DownloadName = "SDL2_ttf"
    },
    @{
        Repo = "SDL_image"
        Version = "2.8.2"
        DownloadName = "SDL2_image"
    },
    @{
        Repo = "SDL_mixer"
        Version = "2.8.0"
        DownloadName = "SDL2_mixer"
    }
)

foreach ($dep in $SDLDepsList) {
    $ParentUrl = "$SDLRootUrl/$($dep.Repo)/releases/download/release-$($dep.Version)"
    $Name = "$($dep.DownloadName)$middleName$($dep.Version)$lastName.zip"

    Write-Output "Setup $($dep.Repo)"

    $zipPath = "$depsDir\$Name"
    $destDir = $zipPath -replace '.zip$'

    if (!(Test-Path -PathType Leaf $zipPath)) {
        Invoke-WebRequest -Uri "$ParentUrl/$Name" -OutFile $zipPath -UseBasicParsing -PassThru | Out-Null

        if (!(Test-Path -PathType Leaf $zipPath)) {
            Write-Output "Download failed"
            exit
        }
    }

    if (Test-Path -PathType Container $destDir) {
        Remove-Item -Recurse -Force -Path $destDir
    }

    Expand-Archive -Path $zipPath -DestinationPath $destDir
    
    if ($libProfile -eq 1) {
        foreach ($children in Get-ChildItem "$destDir\*\*") {
            if (Test-Path -PathType Container $children.FullName) {
                Copy-Item -Recurse -Force $children $depsLibsDir
            }
        }
    } else {
        Copy-Item -Recurse -Force -Exclude "README.txt" "$destDir\*" $depsLibsDir
    }
    Remove-Item -Recurse $destDir
}