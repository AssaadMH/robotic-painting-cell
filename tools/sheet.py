#!/usr/bin/env python3
"""Contact sheet of frames: sheet.py out.jpg frame1.jpg [frame2.jpg ...] (first one big, others in a row)."""
import sys
from PIL import Image
out, files = sys.argv[1], sys.argv[2:]
ims = [Image.open(f) for f in files]
W = 1280
rest = ims[1:]
h_small = int(W / max(len(rest), 1) * 9 / 16) if rest else 0
s = Image.new('RGB', (W, 720 + h_small))
s.paste(ims[0].resize((W, 720)), (0, 0))
for i, im in enumerate(rest):
    w = W // len(rest)
    s.paste(im.resize((w, h_small)), (i * w, 720))
s.save(out, quality=90)
print(out)
