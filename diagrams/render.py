"""Regenerate editable SVGs from example data (standard library only)."""
from pathlib import Path
import sys
from xml.sax.saxutils import escape

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent))
from joint_speed import plan, read_axes


def chart(filename, title, summary, traces, xrange, yrange, xticks, yticks,
          xlabel, ylabel, xmarkers=()):
    left, right, top, bottom = 96, 758, 113, 404
    sx = lambda t: left + (right-left)*(t-xrange[0])/(xrange[1]-xrange[0])
    sy = lambda y: bottom - (bottom-top)*(y-yrange[0])/(yrange[1]-yrange[0])
    parts = [f'<svg xmlns="http://www.w3.org/2000/svg" width="800" height="504" viewBox="0 0 800 504" role="img" aria-labelledby="title desc"><title id="title">{escape(title)}</title><desc id="desc">{escape(summary)}</desc>',
             '<rect width="800" height="504" rx="24" fill="#f2f9fb"/>',
             '<g font-family="DejaVu Sans,sans-serif" fill="#122e45">',
             f'<text x="37" y="46" font-size="27" font-weight="700">{escape(title)}</text>',
             f'<text x="38" y="78" font-size="19">{escape(summary)}</text>']
    for v, label in yticks:
        y = sy(v)
        color = "#718a98" if v == 0 else "#c8dce3"
        parts.append(f'<path d="M{left} {y:.2f}H{right}" stroke="{color}"/><text x="{left-10}" y="{y+6:.2f}" font-size="18" text-anchor="end">{label}</text>')
    for v, label in xticks:
        x = sx(v)
        parts.append(f'<path d="M{x:.2f} {bottom}v7" stroke="#718a98"/><text x="{x:.2f}" y="{bottom+27}" font-size="18" text-anchor="middle">{label}</text>')
    for v in xmarkers:
        x = sx(v)
        parts.append(f'<path d="M{x:.2f} {top}V{bottom}" stroke="#7a929c" stroke-dasharray="5 5"/>')
    parts.append(f'<path d="M{left} {top}V{bottom}H{right}" fill="none" stroke="#718a98" stroke-width="1.5"/>')
    for i, (points, color, dashed, label) in enumerate(traces):
        poly = ' '.join(f'{sx(x):.2f},{sy(y):.2f}' for x, y in points)
        dash = ' stroke-dasharray="10 6"' if dashed else ''
        parts.append(f'<polyline points="{poly}" fill="none" stroke="{color}" stroke-width="4" stroke-linecap="round" stroke-linejoin="round"{dash}/>')
        x = 112 + i*332
        parts.append(f'<path d="M{x} 476h35" stroke="{color}" stroke-width="4"{dash}/><text x="{x+47}" y="482" font-size="18">{escape(label)}</text>')
    parts.extend((f'<text x="428" y="455" text-anchor="middle" font-size="18">{escape(xlabel)}</text>',
                  f'<text transform="translate(25 260) rotate(-90)" text-anchor="middle" font-size="18">{escape(ylabel)}</text>',
                  '</g></svg>'))
    (HERE / filename).write_text('\n'.join(parts)+'\n', encoding="utf-8")


def main():
    m = plan(read_axes(HERE.parent / "examples" / "two-caps.json"))
    moments = [0, m.ramp, m.ramp+m.cruise, m.duration]
    chart("joint-speed-two-caps.svg", "Two joints, one finish",
          "The elbow sets the cruise cap; the shoulder sets the ramp.",
          [([(t,m.sample(t)[i][1]) for t in moments], color, bool(i), label)
           for i,(color,label) in enumerate([("#066f78","Shoulder: +60 deg/s"),("#a84c18","Elbow: −30 deg/s")])],
          (0,m.duration),(-40,70),[(t,f"{t:.2f}") for t in moments],
          [(v,str(v)) for v in (-30,0,30,60)],"Time (s)","Joint speed (deg/s)",moments[1:-1])
    m = plan(read_axes(HERE.parent / "examples" / "short-move.json"))
    angles = [m.sample(i*m.duration/20) for i in range(21)]
    chart("joint-speed-shared-path.svg","Shared progress fixes the path",
          "The shoulder angle increases as the elbow angle decreases.",
          [([(v[0][0],v[1][0]) for v in angles],"#066f78",False,"Same path fraction"),
           ([(10,5),(22,5),(22,-1)],"#a84c18",True,"Shoulder first, elbow second")],
          (9,23),(-2,6),[(v,str(v)) for v in (10,13,16,19,22)],
          [(v,str(v)) for v in (-1,1,3,5)],"Shoulder angle (deg)","Elbow angle (deg)")


if __name__ == "__main__":
    main()
