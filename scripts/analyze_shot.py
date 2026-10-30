#!/usr/bin/env python3
"""Analyze a mech viewer screenshot PPM: what got drawn, where, in what colors."""
import sys

def load_ppm(path):
    with open(path, 'rb') as f:
        magic = f.readline().strip()
        assert magic == b'P6', magic
        # skip comments
        line = f.readline()
        while line.startswith(b'#'):
            line = f.readline()
        w, h = map(int, line.split())
        maxv = int(f.readline())
        data = f.read(w * h * 3)
    return w, h, data

def main(path):
    w, h, data = load_ppm(path)
    px = [(data[i], data[i+1], data[i+2]) for i in range(0, len(data), 3)]

    # background: (14, 17, 20) = kBg 0.055/0.065/0.080 * 255
    def near(c, t, tol=6):
        return all(abs(a-b) <= tol for a, b in zip(c, t))

    bg = (14, 17, 20)
    n_bg = sum(1 for p in px if near(p, bg, 5))
    total = w * h

    # palette colors at full strength: amber(232,176,74) steel(112,179,207)
    # brick(202,112,112) sage(158,209,138) lilac(179,158,219) white(255,255,255,0.92->235)
    targets = {
        'amber': (232, 176, 74), 'steel': (112, 179, 207), 'brick': (202, 112, 112),
        'sage': (158, 209, 138), 'lilac': (179, 158, 219), 'white': (235, 235, 235),
        'gray-ground': (153, 163, 174),
    }
    counts = {k: 0 for k in targets}
    minx, miny, maxx, maxy = w, h, -1, -1
    for i, p in enumerate(px):
        if near(p, bg, 5):
            continue
        # non-background content: track bbox (ignore faint grid: alpha .045 over bg ~ (24,27,30))
        if p[0] > 40 or p[1] > 40 or p[2] > 45:
            x, y = i % w, i // w
            minx, miny = min(minx, x), min(miny, y)
            maxx, maxy = max(maxx, x), max(maxy, y)
        for k, t in targets.items():
            if near(p, t, 28):
                counts[k] += 1
                break

    print(f"image: {w}x{h}, {total} px")
    print(f"background-ish: {n_bg} ({100.0*n_bg/total:.1f}%)")
    print(f"content bbox (non-bg, non-grid): x {minx}..{maxx}, y {miny}..{maxy}"
          f" -> {maxx-minx+1}x{maxy-miny+1} px, centered at "
          f"({(minx+maxx)/2:.0f},{(miny+maxy)/2:.0f})")
    for k, v in counts.items():
        print(f"  {k:12s} {v:8d} px")

if __name__ == '__main__':
    main(sys.argv[1])
