# Bring the RViz (WSLg) window to the front and capture it to a PNG.  usage: grab_rviz.ps1 out.png [title-substring]
param([string]$Out, [string]$Title = "RViz (Ubuntu-24.04)", [int]$Settle = 1500)
Add-Type -AssemblyName System.Drawing
if (-not ('WG' -as [type])) {
Add-Type @"
using System; using System.Runtime.InteropServices; using System.Text;
public class WG {
  [DllImport("user32.dll")] public static extern bool EnumWindows(EnumWindowsProc f, IntPtr p);
  public delegate bool EnumWindowsProc(IntPtr h, IntPtr p);
  [DllImport("user32.dll")] public static extern int GetWindowText(IntPtr h, StringBuilder s, int n);
  [DllImport("user32.dll")] public static extern bool GetWindowRect(IntPtr h, out RECT r);
  [DllImport("user32.dll")] public static extern bool SetForegroundWindow(IntPtr h);
  [DllImport("user32.dll")] public static extern bool ShowWindow(IntPtr h, int cmd);
  [DllImport("user32.dll")] public static extern bool BringWindowToTop(IntPtr h);
  [DllImport("user32.dll")] public static extern bool SetProcessDPIAware();
  [StructLayout(LayoutKind.Sequential)] public struct RECT { public int L,T,R,B; }
  public static IntPtr Find(string sub){ IntPtr f=IntPtr.Zero; EnumWindows((h,p)=>{ var sb=new StringBuilder(512); GetWindowText(h,sb,512); if(sb.ToString().Contains(sub)) f=h; return true;},IntPtr.Zero); return f; }
}
"@ }
[WG]::SetProcessDPIAware() | Out-Null
$h = [WG]::Find($Title)
if ($h -eq [IntPtr]::Zero) { "window not found"; exit 1 }
[WG]::ShowWindow($h, 9) | Out-Null
[WG]::BringWindowToTop($h) | Out-Null
[WG]::SetForegroundWindow($h) | Out-Null
Start-Sleep -Milliseconds $Settle
$r = New-Object WG+RECT; [WG]::GetWindowRect($h, [ref]$r) | Out-Null
$w = $r.R - $r.L; $hh = $r.B - $r.T
$bmp = New-Object System.Drawing.Bitmap $w, $hh
$g = [System.Drawing.Graphics]::FromImage($bmp)
$g.CopyFromScreen($r.L, $r.T, 0, 0, $bmp.Size)
$bmp.Save($Out, [System.Drawing.Imaging.ImageFormat]::Png)
"saved $Out at $($r.L),$($r.T) ${w}x${hh}"
