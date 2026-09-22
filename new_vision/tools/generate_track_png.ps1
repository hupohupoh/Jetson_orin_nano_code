param(
    [string]$OutputPath = ''
)

$ErrorActionPreference = 'Stop'

Add-Type -AssemblyName System.Drawing

$repoRoot = Split-Path -Parent $PSScriptRoot
$dxfPath = Join-Path $repoRoot 'dxf_out\2025人形竞技全能场地图纸.dxf'
$jsonPath = Join-Path $repoRoot 'config\field\场地参数基线.json'
if ([string]::IsNullOrWhiteSpace($OutputPath)) {
    $outPath = Join-Path $repoRoot 'generated\track_competition_from_dxf.png'
}
else {
    if ([System.IO.Path]::IsPathRooted($OutputPath)) {
        $outPath = $OutputPath
    }
    else {
        $outPath = Join-Path $repoRoot $OutputPath
    }
}

$outDir = Split-Path -Parent $outPath
if (!(Test-Path $outDir)) {
    New-Item -ItemType Directory -Path $outDir | Out-Null
}

if (!(Test-Path $dxfPath)) { throw "DXF not found: $dxfPath" }
if (!(Test-Path $jsonPath)) { throw "JSON not found: $jsonPath" }

$data = Get-Content $jsonPath -Raw | ConvertFrom-Json
$minX = [double]$data.field.border_rect.min[0]
$minY = [double]$data.field.border_rect.min[1]
$maxX = [double]$data.field.border_rect.max[0]
$maxY = [double]$data.field.border_rect.max[1]
$fieldW = [double]$data.field.width
$fieldH = [double]$data.field.height

$scale = 1.0 # px per mm
$margin = 80
$bmpW = [int][Math]::Round($fieldW * $scale + 2 * $margin)
$bmpH = [int][Math]::Round($fieldH * $scale + 2 * $margin)

function Parse-LwPolylinesFromEntities {
    param([string[]]$lines)

    $inEntities = $false
    $i = 0
    $results = @()

    while ($i -lt ($lines.Count - 1)) {
        $code = $lines[$i].Trim()
        $val = $lines[$i + 1].Trim()

        if ($code -eq '2' -and $val -eq 'ENTITIES') {
            $inEntities = $true
            $i += 2
            continue
        }
        if ($inEntities -and $code -eq '0' -and $val -eq 'ENDSEC') {
            break
        }

        if ($inEntities -and $code -eq '0' -and $val -eq 'LWPOLYLINE') {
            $entity = @()
            $j = $i + 2
            while ($j -lt ($lines.Count - 1)) {
                $c = $lines[$j].Trim()
                $v = $lines[$j + 1].Trim()
                if ($c -eq '0') { break }
                $entity += @($c, $v)
                $j += 2
            }

            $closed = $false
            $verts = New-Object System.Collections.Generic.List[object]
            $current = $null

            for ($k = 0; $k -lt $entity.Count; $k += 2) {
                $c = $entity[$k]
                $v = $entity[$k + 1]

                if ($c -eq '70') {
                    $flags = [int]$v
                    $closed = (($flags -band 1) -ne 0)
                }
                elseif ($c -eq '10') {
                    if ($null -ne $current) {
                        [void]$verts.Add($current)
                    }
                    $current = [ordered]@{
                        x = [double]$v
                        y = $null
                        bulge = 0.0
                    }
                }
                elseif ($c -eq '20' -and $null -ne $current) {
                    $current.y = [double]$v
                }
                elseif ($c -eq '42' -and $null -ne $current) {
                    $current.bulge = [double]$v
                }
            }
            if ($null -ne $current) {
                [void]$verts.Add($current)
            }

            if ($verts.Count -gt 2 -and $closed) {
                $results += ,([pscustomobject]@{ closed = $closed; vertices = $verts })
            }
            $i = $j
            continue
        }

        $i += 2
    }

    return $results
}

function Parse-EntityExtras {
    param([string[]]$lines)

    $inEntities = $false
    $i = 0
    $extraLines = @()
    $extraTexts = @()

    while ($i -lt ($lines.Count - 1)) {
        $code = $lines[$i].Trim()
        $val = $lines[$i + 1].Trim()

        if ($code -eq '2' -and $val -eq 'ENTITIES') {
            $inEntities = $true
            $i += 2
            continue
        }
        if ($inEntities -and $code -eq '0' -and $val -eq 'ENDSEC') {
            break
        }

        if (-not $inEntities) {
            $i += 2
            continue
        }

        if ($code -eq '0' -and $val -eq 'LINE') {
            $j = $i + 2
            $x1 = $null; $y1 = $null; $x2 = $null; $y2 = $null
            while ($j -lt ($lines.Count - 1)) {
                $c = $lines[$j].Trim()
                $v = $lines[$j + 1].Trim()
                if ($c -eq '0') { break }
                if ($c -eq '10') { $x1 = [double]$v }
                elseif ($c -eq '20') { $y1 = [double]$v }
                elseif ($c -eq '11') { $x2 = [double]$v }
                elseif ($c -eq '21') { $y2 = [double]$v }
                $j += 2
            }
            if ($null -ne $x1 -and $null -ne $y1 -and $null -ne $x2 -and $null -ne $y2) {
                $len = [Math]::Sqrt(($x2 - $x1) * ($x2 - $x1) + ($y2 - $y1) * ($y2 - $y1))
                if ($len -gt 1e-3) {
                    $extraLines += ,([pscustomobject]@{ x1 = $x1; y1 = $y1; x2 = $x2; y2 = $y2 })
                }
            }
            $i = $j
            continue
        }

        if ($code -eq '0' -and ($val -eq 'TEXT' -or $val -eq 'MTEXT')) {
            $j = $i + 2
            $x = $null; $y = $null; $txt = ''
            while ($j -lt ($lines.Count - 1)) {
                $c = $lines[$j].Trim()
                $v = $lines[$j + 1].Trim()
                if ($c -eq '0') { break }
                if ($c -eq '10') { $x = [double]$v }
                elseif ($c -eq '20') { $y = [double]$v }
                elseif ($c -eq '1' -or $c -eq '3') {
                    if ($txt -eq '') { $txt = $v } else { $txt += $v }
                }
                $j += 2
            }
            if ($null -ne $x -and $null -ne $y -and $txt -ne '') {
                $extraTexts += ,([pscustomobject]@{ x = $x; y = $y; text = $txt })
            }
            $i = $j
            continue
        }

        $i += 2
    }

    return [pscustomobject]@{ lines = $extraLines; texts = $extraTexts }
}

function Convert-PolylineToPoints {
    param(
        [object]$polyline,
        [double]$arcStepMm = 20.0
    )

    $pts = New-Object System.Collections.Generic.List[object]
    $verts = $polyline.vertices
    $n = $verts.Count
    if ($n -lt 2) { return $pts }

    for ($i = 0; $i -lt $n; $i++) {
        $j = ($i + 1) % $n
        if (-not $polyline.closed -and $i -eq $n - 1) { break }

        $p1 = $verts[$i]
        $p2 = $verts[$j]
        $x1 = [double]$p1.x
        $y1 = [double]$p1.y
        $x2 = [double]$p2.x
        $y2 = [double]$p2.y
        $b = [double]$p1.bulge

        if ([Math]::Abs($b) -lt 1e-9) {
            if ($pts.Count -eq 0) {
                [void]$pts.Add([pscustomobject]@{ x = $x1; y = $y1 })
            }
            [void]$pts.Add([pscustomobject]@{ x = $x2; y = $y2 })
            continue
        }

        $dx = $x2 - $x1
        $dy = $y2 - $y1
        $c = [Math]::Sqrt($dx * $dx + $dy * $dy)
        if ($c -lt 1e-6) { continue }

        $theta = 4.0 * [Math]::Atan($b)
        $mx = 0.5 * ($x1 + $x2)
        $my = 0.5 * ($y1 + $y2)
        $ux = $dx / $c
        $uy = $dy / $c
        $nx = -$uy
        $ny = $ux

        $d = $c * (1.0 - $b * $b) / (4.0 * $b)
        $cx = $mx + $d * $nx
        $cy = $my + $d * $ny
        $r = [Math]::Sqrt(($x1 - $cx) * ($x1 - $cx) + ($y1 - $cy) * ($y1 - $cy))

        $steps = [Math]::Max(8, [int][Math]::Ceiling(([Math]::Abs($theta) * $r) / $arcStepMm))
        $a0 = [Math]::Atan2($y1 - $cy, $x1 - $cx)

        if ($pts.Count -eq 0) {
            [void]$pts.Add([pscustomobject]@{ x = $x1; y = $y1 })
        }

        for ($k = 1; $k -le $steps; $k++) {
            $t = $k / [double]$steps
            $a = $a0 + $theta * $t
            $px = $cx + $r * [Math]::Cos($a)
            $py = $cy + $r * [Math]::Sin($a)
            [void]$pts.Add([pscustomobject]@{ x = $px; y = $py })
        }
    }

    return $pts
}

function ToPixel {
    param([double]$x, [double]$y)
    $px = $margin + ($x - $minX) * $scale
    $py = $margin + ($maxY - $y) * $scale
    return New-Object System.Drawing.PointF([float]$px, [float]$py)
}

function Draw-ConfiguredSegment {
    param(
        [System.Drawing.Graphics]$Graphics,
        [object]$Segment,
        [string]$ColorName,
        [string]$DashName,
        [double]$Width = 6.0
    )

    if ($null -eq $Segment -or $Segment.Count -lt 2) { return }

    $p0 = ToPixel -x ([double]$Segment[0][0]) -y ([double]$Segment[0][1])
    $p1 = ToPixel -x ([double]$Segment[1][0]) -y ([double]$Segment[1][1])

    $color = [System.Drawing.Color]::Black
    if ($ColorName -eq 'red') {
        $color = [System.Drawing.Color]::FromArgb(220, 40, 40)
    }
    elseif ($ColorName -eq 'gray') {
        $color = [System.Drawing.Color]::FromArgb(90, 90, 90)
    }

    $pen = New-Object System.Drawing.Pen($color, [float]$Width)
    if ($DashName -eq 'dash') {
        $pen.DashStyle = [System.Drawing.Drawing2D.DashStyle]::Dash
    }
    else {
        $pen.DashStyle = [System.Drawing.Drawing2D.DashStyle]::Solid
    }

    $Graphics.DrawLine($pen, $p0, $p1)
    $pen.Dispose()
}

$lines = Get-Content $dxfPath
$polys = Parse-LwPolylinesFromEntities -lines $lines
$extras = Parse-EntityExtras -lines $lines
if ($polys.Count -lt 2) {
    throw "Expected at least 2 closed LWPOLYLINE entities, got $($polys.Count)"
}

$polyPoints = @()
foreach ($p in $polys) {
    $mmPts = Convert-PolylineToPoints -polyline $p
    if ($mmPts.Count -gt 3) {
        $pixPts = New-Object System.Collections.Generic.List[System.Drawing.PointF]
        foreach ($pt in $mmPts) {
            [void]$pixPts.Add((ToPixel -x $pt.x -y $pt.y))
        }
        $polyPoints += ,$pixPts
    }
}

$bmp = New-Object System.Drawing.Bitmap($bmpW, $bmpH)
$g = [System.Drawing.Graphics]::FromImage($bmp)
$g.SmoothingMode = [System.Drawing.Drawing2D.SmoothingMode]::AntiAlias
$g.Clear([System.Drawing.Color]::White)

$borderPen = New-Object System.Drawing.Pen([System.Drawing.Color]::FromArgb(180, 180, 180), 2)
$g.DrawRectangle($borderPen, [float]$margin, [float]$margin, [float]($fieldW * $scale), [float]($fieldH * $scale))

$trackPen = New-Object System.Drawing.Pen([System.Drawing.Color]::Black, 12)
foreach ($pp in $polyPoints) {
    if ($pp.Count -gt 1) {
        $arr = $pp.ToArray()
        $g.DrawLines($trackPen, $arr)
    }
}

$extraLinePen = New-Object System.Drawing.Pen([System.Drawing.Color]::FromArgb(110, 110, 110), 2)
foreach ($ln in $extras.lines) {
    $pA = ToPixel -x ([double]$ln.x1) -y ([double]$ln.y1)
    $pB = ToPixel -x ([double]$ln.x2) -y ([double]$ln.y2)
    $g.DrawLine($extraLinePen, $pA, $pB)
}

if ($null -ne $data.elements.obstacle.line_segment) {
    Draw-ConfiguredSegment -Graphics $g -Segment $data.elements.obstacle.line_segment -ColorName $data.elements.obstacle.color -DashName $data.elements.obstacle.dash -Width 8.0
}

if ($null -ne $data.elements.start_line.line_segment) {
    Draw-ConfiguredSegment -Graphics $g -Segment $data.elements.start_line.line_segment -ColorName $data.elements.start_line.color -DashName $data.elements.start_line.dash -Width 8.0
}
elseif ($null -ne $data.elements.start_line.candidate_segment) {
    Draw-ConfiguredSegment -Graphics $g -Segment $data.elements.start_line.candidate_segment -ColorName 'black' -DashName 'solid' -Width 8.0
}

$font = New-Object System.Drawing.Font('Microsoft YaHei', 18, [System.Drawing.FontStyle]::Bold)
$brush = New-Object System.Drawing.SolidBrush([System.Drawing.Color]::FromArgb(40, 40, 40))
$g.DrawString('RoboCup Track (from DXF)', $font, $brush, 20, 20)

$markerFont = New-Object System.Drawing.Font('Microsoft YaHei', 16, [System.Drawing.FontStyle]::Regular)
foreach ($tx in $extras.texts) {
    if ($tx.text -match '^[0-9]+$') {
        $tp = ToPixel -x ([double]$tx.x) -y ([double]$tx.y)
        $g.DrawString($tx.text, $markerFont, $brush, $tp.X - 8, $tp.Y - 8)
    }
}

$bmp.Save($outPath, [System.Drawing.Imaging.ImageFormat]::Png)

$brush.Dispose()
$markerFont.Dispose()
$font.Dispose()
$extraLinePen.Dispose()
$trackPen.Dispose()
$borderPen.Dispose()
$g.Dispose()
$bmp.Dispose()

Write-Output "Generated: $outPath"
