#!/usr/bin/env python3
"""Fill in the personal details everywhere and build the submission folder (Windows Python).
usage: finalize.py --name "Full Name" --college "..." --email "..." --repo URL --drive URL [--skip-video]
Creates C:/Users/HP/yuktii/<Name_With_Underscores>/ with the structure required by the brief:
   Technical_Report.pdf  Simulation_Files/  Screenshots/  Demo_Video/  GitHub_Link.txt"""
import argparse, os, re, shutil, subprocess, sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
ap = argparse.ArgumentParser()
ap.add_argument('--name', required=True); ap.add_argument('--college', required=True); ap.add_argument('--email', required=True)
ap.add_argument('--repo', default='[GitHub link]'); ap.add_argument('--drive', default='[Google Drive link]')
ap.add_argument('--skip-video', action='store_true')
a = ap.parse_args()

# 1. README markers
rd = os.path.join(ROOT, 'README.md')
s = open(rd, encoding='utf-8').read()
s = s.replace('[FULL NAME]', a.name).replace('[Google Drive link]', a.drive)
open(rd, 'w', encoding='utf-8').write(s)
print('README updated')

# 2. report (figures + Gazebo summary from the render first)
subprocess.run(['wsl.exe', '-d', 'Ubuntu-24.04', '--', 'python3', '/mnt/c/Users/HP/yuktii/painting_cell/tools/gz_figures.py'], check=True)
subprocess.run([sys.executable, os.path.join(ROOT, 'tools', 'build_report.py'), '--name', a.name, '--college', a.college,
                '--email', a.email, '--repo', a.repo, '--drive', a.drive], check=True)

# 3. video (WSL): title card + last card carry the name and links
if not a.skip_video:
    cmd = 'python3 /mnt/c/Users/HP/yuktii/painting_cell/tools/make_video_gz.py --name "%s" --college "%s" --repo "%s" --drive "%s"' % (
              a.name, a.college, a.repo.replace('https://', ''), a.drive.replace('https://', ''))
    subprocess.run(['wsl.exe', '-d', 'Ubuntu-24.04', '--', 'bash', '-c', cmd], check=True)

# 4. submission folder
safe = re.sub(r'[^A-Za-z0-9]+', '_', a.name).strip('_')
out = os.path.join(os.path.dirname(ROOT), safe)
if os.path.exists(out):
    shutil.rmtree(out)
os.makedirs(out)
shutil.copy(os.path.join(ROOT, 'docs', 'Technical_Report.pdf'), os.path.join(out, 'Technical_Report.pdf'))
shutil.copytree(os.path.join(ROOT, 'docs', 'screenshots'), os.path.join(out, 'Screenshots'))
os.makedirs(os.path.join(out, 'Demo_Video'))
mp4 = os.path.join(ROOT, 'video', 'demo.mp4')
if os.path.exists(mp4):
    shutil.copy(mp4, os.path.join(out, 'Demo_Video', 'demo.mp4'))
sim = os.path.join(out, 'Simulation_Files')
ignore = shutil.ignore_patterns('__pycache__', '*.pyc', 'frames', 'preview', 'gz_composed', 'captions', 'cards', '*.wav', 'concat.txt', '*.mp4', '.git', 'report.html', 'v1_box_car*', '_*.py', '_*.sh',
                               'car_*.stl', 'car_*.obj', 'paint_*.obj')    # third-party-derived car meshes stay out
for d in ('src', 'tools', 'results', 'video'):
    shutil.copytree(os.path.join(ROOT, d), os.path.join(sim, d), ignore=ignore)
for f in ('README.md', 'LICENSE', 'NOTICE', '.gitignore'):
    shutil.copy(os.path.join(ROOT, f), os.path.join(sim, f))
open(os.path.join(out, 'GitHub_Link.txt'), 'w').write(a.repo + '\n')
print('submission folder:', out)
for dp, dn, fn in os.walk(out):
    if dp.count(os.sep) - out.count(os.sep) <= 1:
        print('  ', os.path.relpath(dp, out), [f for f in fn][:6])
