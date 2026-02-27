using System;
using System.Linq;
using System.Numerics;

public static Vector3[] OrderCornersTLTRBRBL(ReadOnlySpan<Vector3> corners)
{
    if (corners.Length != 4) throw new ArgumentException("Need 4 corners.");
    var pts = corners.ToArray();
    var c = (pts[0] + pts[1] + pts[2] + pts[3]) * 0.25f;

    var n = Vector3.Cross(pts[1] - pts[0], pts[3] - pts[0]);
    n = n.LengthSquared() > 1e-12f ? Vector3.Normalize(n) : Vector3.UnitZ;

    var u = pts.Select(p => { var d = p - c; d -= n * Vector3.Dot(d, n); return d; })
               .OrderByDescending(d => d.LengthSquared()).First();
    u = u.LengthSquared() > 1e-12f ? Vector3.Normalize(u) : Vector3.UnitX;
    var v = Vector3.Normalize(Vector3.Cross(n, u));

    var p2 = pts.Select(p => { var d = p - c; var x = Vector3.Dot(d, u); var y = Vector3.Dot(d, v); return (p, x, y, s: x + y, d2: x - y); }).ToArray();

    var tl = p2.OrderBy(t => t.d2).First().p;
    var tr = p2.OrderByDescending(t => t.s).First().p;
    var br = p2.OrderByDescending(t => t.d2).First().p;
    var bl = p2.OrderBy(t => t.s).First().p;

    // If degenerate/tied and duplicates occur, fall back to CCW angle ordering then pick TL as highest Y then lowest X.
    if (new[] { tl, tr, br, bl }.Distinct().Count() != 4)
    {
        var ccw = p2.OrderBy(t => MathF.Atan2(t.y, t.x)).ToArray();
        var tlIdx = ccw.Select((t, i) => (t, i)).OrderByDescending(t => t.t.y).ThenBy(t => t.t.x).First().i;
        return new[] { ccw[tlIdx].p, ccw[(tlIdx + 1) & 3].p, ccw[(tlIdx + 2) & 3].p, ccw[(tlIdx + 3) & 3].p };
    }

    return new[] { tl, tr, br, bl };
}