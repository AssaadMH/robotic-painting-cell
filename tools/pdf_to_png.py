import sys, fitz
pdf, outdir = sys.argv[1], sys.argv[2]
doc = fitz.open(pdf)
for i, page in enumerate(doc):
    pix = page.get_pixmap(dpi=100)
    pix.save('%s/page%d.png' % (outdir, i + 1))
print('pages', len(doc))
