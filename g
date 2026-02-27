using System;
using System.Linq;
using System.Numerics;

public static class FiducialOrder
{
    public static Vector3[] OrderCornersTLTRBRBL(ReadOnlySpan<Vector3> corners)
    {
        if (corners.Length != 4) throw new ArgumentException("Need 4 corners.");

        var pts = corners.ToArray();
        var c = (pts[0] + pts[1] + pts[2] + pts[3]) * 0.25f;

        var n = Vector3.Normalize(Vector3.Cross(pts[1] - pts[0], pts[3] - pts[0]));
        var u = pts.Select(p => { var d = p - c; d -= n * Vector3.Dot(d, n); return d; })
                   .OrderByDescending(d => d.LengthSquared()).First();
        u = u.LengthSquared() > 1e-12f ? Vector3.Normalize(u) : Vector3.UnitX;
        var v = Vector3.Normalize(Vector3.Cross(n, u));

        var p2 = pts.Select(p => { var d = p - c; return (p, x: Vector3.Dot(d, u), y: Vector3.Dot(d, v)); }).ToArray();
        float midX = (p2.Min(t => t.x) + p2.Max(t => t.x)) * 0.5f, midY = (p2.Min(t => t.y) + p2.Max(t => t.y)) * 0.5f;

        Vector3 Pick(Func<(Vector3 p, float x, float y), bool> pred) => p2.Where(pred).OrderByDescending(t => t.x * t.x + t.y * t.y).First().p;

        var tl = Pick(t => t.x <= midX && t.y >= midY);
        var tr = Pick(t => t.x >= midX && t.y >= midY);
        var br = Pick(t => t.x >= midX && t.y <= midY);
        var bl = Pick(t => t.x <= midX && t.y <= midY);

        return new[] { tl, tr, br, bl };
    }
}