using System;
using System.Collections.Generic;
using System.Linq;
using System.Numerics;

public static Vector3[] OrderCornersFromKnownOrigin(
    List<ScanPointXYZ> cornersPts,
    Vector3 knownOrigin,
    Vector3 xHint)
{
    if (cornersPts == null || cornersPts.Count != 4)
        throw new ArgumentException("Need 4 corners.");

    var pts = cornersPts.Select(p => p.ToVector3()).ToArray();

    // Pick detected corner nearest the known origin
    int iOrigin = Enumerable.Range(0, 4)
        .OrderBy(i => Vector3.DistanceSquared(pts[i], knownOrigin))
        .First();

    var origin = pts[iOrigin];

    var others = Enumerable.Range(0, 4)
        .Where(i => i != iOrigin)
        .Select(i => pts[i])
        .ToArray();

    // Opposite corner is farthest from origin
    var opposite = others
        .OrderByDescending(p => Vector3.DistanceSquared(p, origin))
        .First();

    // Remaining two are adjacent corners
    var adj = others.Where(p => p != opposite).ToArray();
    var a = adj[0];
    var b = adj[1];

    // Plane normal from origin + adjacent edges
    var n = Vector3.Cross(a - origin, b - origin);
    n = n.LengthSquared() > 1e-12f ? Vector3.Normalize(n) : Vector3.UnitZ;

    // Project xHint into the fiducial plane
    var xRef = xHint - n * Vector3.Dot(xHint, n);
    xRef = xRef.LengthSquared() > 1e-12f ? Vector3.Normalize(xRef) : Vector3.Normalize(a - origin);

    var da = Vector3.Normalize(a - origin);
    var db = Vector3.Normalize(b - origin);

    // Choose the adjacent corner most aligned with +X
    Vector3 xNeighbor, yNeighbor;
    if (Vector3.Dot(da, xRef) >= Vector3.Dot(db, xRef))
    {
        xNeighbor = a;
        yNeighbor = b;
    }
    else
    {
        xNeighbor = b;
        yNeighbor = a;
    }

    return new[] { origin, xNeighbor, opposite, yNeighbor };
}