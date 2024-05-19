$dataDir = "$PSScriptRoot\..\Data"

$xml = [xml](Get-Content "$dataDir\Map.svg")

$width = $xml.svg.width
$height = $xml.svg.height

$output = @"
width: $width
height: $height
entityList: {
"@;

foreach ($image in $xml.svg.image) {
    $centerX = ([double] $image.x) + ([double] $image.width) / 2
    $centerY = ([double] $image.y) + ([double] $image.height) / 2
    if ($image.href.Contains("Meteors")) {
        $type = $image.href -replace "^PNG/Meteors/meteor","" -replace ".png$",""
        $entityData = @"
  Meteor {
    x: $($centerX)
    y: $($centerY)
    type: $($type) 
  }
"@
        $output = @"
$output
$entityData
"@
    } else {
        Write-Output "Not supported :"
        Write-Output $image
    }
}

$output = @"
$output
}
"@

Write-Output $output | Out-File "$dataDir\Map.data"