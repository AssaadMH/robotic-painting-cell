# Move/resize the RViz (WSLg) window and bring it to the front. usage: place_rviz.ps1 [x y w h]
param([int]$X = 0, [int]$Y = 0, [int]$W = 1920, [int]$H = 1080, [string]$Title = "RViz (Ubuntu-24.04)")
if (-not ('WP' -as [type])) {
Add-Type @"
using System; using System.Runtime.InteropServices; using System.Text;
public class WP {
  [DllImport("user32.dll")] public static extern bool EnumWindows(EnumWindowsProc f, IntPtr p);
  public delegate bool EnumWindowsProc(IntPtr h, IntPtr p);
  [DllImport("user32.dll")] public static extern int GetWindowText(IntPtr h, StringBuilder s, int n);
  [DllImport("user32.dll")] public static extern bool MoveWindow(IntPtr h, int x, int y, int w, int hh, bool r);
  [DllImport("user32.dll")] public static extern bool SetForegroundWindow(IntPtr h);
  [DllImport("user32.dll")] public static extern bool ShowWindow(IntPtr h, int c);
  [DllImport("user32.dll")] public static extern bool SetProcessDPIAware();
  [DllImport("user32.dll")] public static extern bool GetWindowRect(IntPtr h, out RECT r);
  [StructLayout(LayoutKind.Sequential)] public struct RECT { public int L,T,R,B; }
  public static IntPtr Find(string sub){ IntPtr f=IntPtr.Zero; EnumWindows((h,p)=>{ var sb=new StringBuilder(512); GetWindowText(h,sb,512); if(sb.ToString().Contains(sub)) f=h; return true;},IntPtr.Zero); return f; }
}
"@ }
[WP]::SetProcessDPIAware() | Out-Null
$h = [WP]::Find($Title)
if ($h -eq [IntPtr]::Zero) { "window not found"; exit 1 }
[WP]::ShowWindow($h, 9) | Out-Null
[WP]::MoveWindow($h, $X, $Y, $W, $H, $true) | Out-Null
[WP]::SetForegroundWindow($h) | Out-Null
Start-Sleep -Milliseconds 800
$r = New-Object WP+RECT; [WP]::GetWindowRect($h, [ref]$r) | Out-Null
"window now at $($r.L),$($r.T) size $($r.R-$r.L)x$($r.B-$r.T)"
