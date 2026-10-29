# Synthesise narration.json with the Windows English voice. usage: tts.ps1 narration.json outdir [voice] [rate]
param([string]$Json, [string]$OutDir, [string]$Voice = "Microsoft Zira Desktop", [int]$Rate = 0)
Add-Type -AssemblyName System.Speech
New-Item -ItemType Directory -Force $OutDir | Out-Null
$items = Get-Content -Raw -Encoding UTF8 $Json | ConvertFrom-Json
$fmt = New-Object System.Speech.AudioFormat.SpeechAudioFormatInfo(44100, [System.Speech.AudioFormat.AudioBitsPerSample]::Sixteen, [System.Speech.AudioFormat.AudioChannel]::Mono)
$res = @{}
foreach ($it in $items) {
  $s = New-Object System.Speech.Synthesis.SpeechSynthesizer
  $s.SelectVoice($Voice); $s.Rate = $Rate
  $path = Join-Path $OutDir ($it.id + ".wav")
  $s.SetOutputToWaveFile($path, $fmt)
  $s.Speak($it.text)
  $s.Dispose()
  $len = (Get-Item $path).Length
  $res[$it.id] = [math]::Round(($len - 44) / (44100 * 2), 2)
  "{0}  {1:N1} s" -f $it.id, $res[$it.id]
}
($res | ConvertTo-Json) | Set-Content -Encoding UTF8 (Join-Path $OutDir "durations.json")
