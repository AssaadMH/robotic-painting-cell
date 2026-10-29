import os, sys, time, math, subprocess
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import render_cell as rc
HOME = os.path.expanduser('~')
r = rc.CellRenderer(HOME + '/plan.json', os.path.join(rc.SHARE_CELL, 'paths', 'paths.json'), '/tmp/cell_render.urdf', size=(1280, 720))
cam = rc.orbit_pose(math.radians(-60), math.radians(24), 11.5, (0.3, -0.4, 0.9))
r.render(300.0, cam)
for size in ((1280, 720),):
    t = time.time(); n = 20
    for i in range(n):
        r.render(300.0 + i * 0.5, cam)
    print('steady state %dx%d: %.2f s/frame (%.2f fps)' % (size[0], size[1], (time.time() - t) / n, n / (time.time() - t)))
