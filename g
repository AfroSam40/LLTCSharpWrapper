public static Vector3[] OrderCornersTLTRBRBL(List<ScanPointXYZ> cornersPts)
{
    if (cornersPts == null || cornersPts.Count != 4)
        throw new ArgumentException("Need 4 corners.");

    var pts = cornersPts.Select(pt => pt.ToVector3()).ToArray();
    var c = (pts[0] + pts[1] + pts[2] + pts[3]) * 0.25f;

    // Order-independent normal from all 4 points
    Vector3 n = Vector3.Zero;
    for (int i = 0; i < 4; i++)
    {
        var a = pts[i] - c;
        var b = pts[(i + 1) & 3] - c;
        n += Vector3.Cross(a, b);
    }
    n = n.LengthSquared() > 1e-12f ? Vector3.Normalize(n) : Vector3.UnitZ;

    // Temporary in-plane basis for cyclic ordering
    var u0 = pts[0] - c;
    u0 -= n * Vector3.Dot(u0, n);
    u0 = u0.LengthSquared() > 1e-12f ? Vector3.Normalize(u0) : Vector3.UnitX;
    var v0 = Vector3.Normalize(Vector3.Cross(n, u0));

    // Cyclic order around centroid
    var ccw = pts
        .Select(p =>
        {
            var d = p - c;
            return new
            {
                P = p,
                A = MathF.Atan2(Vector3.Dot(d, v0), Vector3.Dot(d, u0))
            };
        })
        .OrderBy(t => t.A)
        .Select(t => t.P)
        .ToArray();

    // Stable axis from averaged opposite edges
    var e0 = ccw[1] - ccw[0];
    var e1 = ccw[2] - ccw[3];
    var x = e0 + e1;
    if (x.LengthSquared() <= 1e-12f)
        x = e0.LengthSquared() > e1.LengthSquared() ? e0 : e1;
    x = Vector3.Normalize(x);

    var y = Vector3.Normalize(Vector3.Cross(n, x));

    // Project using stable basis
    var p2 = pts.Select(p =>
    {
        var d = p - c;
        float px = Vector3.Dot(d, x);
        float py = Vector3.Dot(d, y);
        return (P: p, X: px, Y: py, S: px + py, D: px - py);
    }).ToArray();

    var tl = p2.OrderBy(t => t.S).First().P;
    var tr = p2.OrderByDescending(t => t.D).First().P;
    var br = p2.OrderByDescending(t => t.S).First().P;
    var bl = p2.OrderBy(t => t.D).First().P;

    // Fallback if classification collides
    if (new[] { tl, tr, br, bl }.Distinct().Count() != 4)
    {
        var cyc = pts
            .Select(p =>
            {
                var d = p - c;
                return new
                {
                    P = p,
                    X = Vector3.Dot(d, x),
                    Y = Vector3.Dot(d, y),
                    A = MathF.Atan2(Vector3.Dot(d, y), Vector3.Dot(d, x))
                };
            })
            .OrderBy(t => t.A)
            .ToArray();

        int tlIdx = cyc.Select((t, i) => (t, i))
            .OrderByDescending(t => t.t.Y)
            .ThenBy(t => t.t.X)
            .First().i;

        return new[]
        {
            cyc[tlIdx].P,
            cyc[(tlIdx + 3) & 3].P,
            cyc[(tlIdx + 2) & 3].P,
            cyc[(tlIdx + 1) & 3].P
        };
    }

    return new[] { tl, tr, br, bl };
}