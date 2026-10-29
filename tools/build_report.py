#!/usr/bin/env python3
"""Fill the report template and print it to PDF with Edge (headless).
usage: build_report.py --name "..." --college "..." --email "..." --repo URL --drive URL"""
import argparse, os, subprocess, sys
ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
ap = argparse.ArgumentParser()
ap.add_argument('--name', default='[FULL NAME]'); ap.add_argument('--college', default='[COLLEGE]')
ap.add_argument('--email', default='[EMAIL]'); ap.add_argument('--repo', default='[GitHub link]')
ap.add_argument('--drive', default='[Google Drive link]')
a = ap.parse_args()
s = open(os.path.join(ROOT, 'docs', 'report_template.html'), encoding='utf-8').read()
import json
gz = os.path.join(ROOT, 'results', 'gz_summary.json')
if os.path.exists(gz):
    g = json.load(open(gz))
    gz_txt = ('%s frames; nozzle distance from the planned path on paint passes: mean %.1f mm, max %.1f mm (all job frames: max %.1f mm); '
              '%s / %s paint strips recoloured' %
              (format(g['frames'], ','), g['nozzle_dev_paint_mm_mean'], g['nozzle_dev_paint_mm_max'], g['nozzle_dev_mm_max'],
               format(g['painted_chunks'], ','), format(g['chunks_total'], ',')))
else:
    gz_txt = 'see results/gz_summary.json'
for k, v in (('{{NAME}}', a.name), ('{{COLLEGE}}', a.college), ('{{EMAIL}}', a.email), ('{{REPO_TXT}}', a.repo.split('://')[-1]), ('{{DRIVE_TXT}}', a.drive.split('://')[-1].split('?')[0]),
                ('{{REPO}}', a.repo), ('{{DRIVE}}', a.drive), ('{{GZ_RESULT}}', gz_txt)):
    s = s.replace(k, v)
html = os.path.join(ROOT, 'docs', 'report.html')
open(html, 'w', encoding='utf-8').write(s)
pdf = os.path.join(ROOT, 'docs', 'Technical_Report.pdf')
edge = next(p for p in (r'C:\Program Files (x86)\Microsoft\Edge\Application\msedge.exe', r'C:\Program Files\Microsoft\Edge\Application\msedge.exe') if os.path.exists(p))
subprocess.run([edge, '--headless=new', '--disable-gpu', '--no-pdf-header-footer', '--print-to-pdf=' + pdf, 'file:///' + html.replace(chr(92), '/')], check=True, timeout=120)
print('wrote', pdf, os.path.getsize(pdf) // 1024, 'KB')
