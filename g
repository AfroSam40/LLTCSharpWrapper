using System;
using System.Linq;
using System.Numerics;

public static class FiducialXform
{
    // Orders 4 corner points into TL, TR, BR, BL using an in-plane 2D projection.
    // Assumes corners are approximately coplanar and form a convex quad (typical rectangular fiducial).
    public static Vector3[] OrderCornersTLTRBRBL(ReadOnlySpan<Vector3> corners)
    {
        if (corners.Length != 4) throw new ArgumentException("Expected exactly 4 corners.");

        var pts = corners.ToArray();
        var c = (pts[0] + pts[1] + pts[2] + pts[3]) * 0.25f;

        // Robust-ish plane normal from summed cross products around the centroid
        Vector3 n = default;
        for (int i = 0; i < 4; i++)
        {
            var a = pts[i] - c;
            var b = pts[(i + 1) & 3] - c;
            n += Vector3.Cross(a, b);
        }
        n = n.LengthSquared() > 1e-12f ? Vector3.Normalize(n) : Vector3.UnitZ;

        // In-plane basis (u,v)
        var u = pts[0] - c;
        u -= n * Vector3.Dot(u, n);
        u = u.LengthSquared() > 1e-12f ? Vector3.Normalize(u) : Vector3.UnitX;
        var v = Vector3.Normalize(Vector3.Cross(n, u));

        // Project to 2D and pick corners by extrema: TL(minX,maxY), TR(maxX,maxY), BR(maxX,minY), BL(minX,minY)
        var proj = pts.Select(p =>
        {
            var d = p - c;
            return (p, x: Vector3.Dot(d, u), y: Vector3.Dot(d, v));
        }).ToArray();

        var tl = proj.OrderBy(t => t.x).ThenByDescending(t => t.y).First().p;
        var tr = proj.OrderByDescending(t => t.x).ThenByDescending(t => t.y).First().p;
        var br = proj.OrderByDescending(t => t.x).ThenBy(t => t.y).First().p;
        var bl = proj.OrderBy(t => t.x).ThenBy(t => t.y).First().p;

        return new[] { tl, tr, br, bl };
    }

    // Builds a full 4x4 transform from fiducial-local coordinates to world:
    // - Rotation from the fiducial edges (TLTRBRBL order)
    // - Translation set to the selected corner (work origin at that corner)
    //
    // cornerOrigin: 0=TL, 1=TR, 2=BR, 3=BL
    public static Matrix4x4 RotationMatrixFrom4Fiducials_WithCornerOriginTranslation(
        ReadOnlySpan<Vector3> cornersTLTRBRBL,
        int cornerOrigin)
    {
        if (cornersTLTRBRBL.Length != 4) throw new ArgumentException("Expected 4 corners: TL,TR,BR,BL.");
        if ((uint)cornerOrigin > 3) throw new ArgumentOutOfRangeException(nameof(cornerOrigin), "cornerOrigin must be 0..3 (TL,TR,BR,BL).");

        var c = cornersTLTRBRBL;

        // Fiducial axes in world from averaged opposing edges
        var top    = c[1] - c[0]; // TR - TL
        var bottom = c[2] - c[3]; // BR - BL
        var left   = c[3] - c[0]; // BL - TL
        var right  = c[2] - c[1]; // BR - TR

        var x = Vector3.Normalize(top + bottom);
        var y0 = Vector3.Normalize(left + right);

        var z = Vector3.Normalize(Vector3.Cross(x, y0));
        var y = Vector3.Normalize(Vector3.Cross(z, x));

        // Translation: choose which fiducial corner is the work-object origin in world
        var t = c[cornerOrigin];

        // Matrix4x4 (System.Numerics): axes in columns, translation in M41..M43
        return new Matrix4x4(
            x.X, y.X, z.X, 0f,
            x.Y, y.Y, z.Y, 0f,
            x.Z, y.Z, z.Z, 0f,
            t.X, t.Y, t.Z, 1f
        );
    }
}