public static double EstimateBlobVolumeBySlices(
    Point3DCollection points,
    PlaneFitResult basePlane,
    double sliceThickness,
    out List<BlobSlice> slices,
    int minPointsPerSlice = 50)
{
    slices = new List<BlobSlice>();
    if (points == null || points.Count == 0) return 0.0;
    if (sliceThickness <= 0) throw new ArgumentOutOfRangeException(nameof(sliceThickness));

    // ---- 1. Build orthonormal basis (u, v, n) ----
    Vector3D n = basePlane.Normal;
    if (n.LengthSquared < 1e-12) return 0.0;

    // Force normal to point "up" in +Z (helps make h > 0 mean "above the plane").
    if (Vector3D.DotProduct(n, new Vector3D(0, 0, 1)) < 0)
    {
        n = -n;
        basePlane.Normal = n;
    }
    n.Normalize();

    // Choose a vector not parallel to n for constructing u
    Vector3D temp = Math.Abs(n.Z) < 0.9
        ? new Vector3D(0, 0, 1)
        : new Vector3D(0, 1, 0);

    Vector3D u = Vector3D.CrossProduct(temp, n);
    if (u.LengthSquared < 1e-12)
        u = new Vector3D(1, 0, 0);
    u.Normalize();

    Vector3D v = Vector3D.CrossProduct(n, u);
    v.Normalize();

    Point3D origin = basePlane.Centroid;

    // ---- 2. Transform points into local (u, v, n) coords; keep only above plane ----
    var localPoints = new List<(double U, double V, double H)>(points.Count);

    foreach (var p in points)
    {
        Vector3D d = p - origin;

        double h  = Vector3D.DotProduct(d, n); // height above plane
        if (h <= 0.0)       // <-- Only keep blob side
            continue;

        double uu = Vector3D.DotProduct(d, u); // in-plane coord
        double vv = Vector3D.DotProduct(d, v);

        localPoints.Add((uu, vv, h));
    }

    if (localPoints.Count == 0)
        return 0.0;

    // ---- 3. Determine height range and slice grid ----
    double minH = localPoints.Min(p => p.H);
    double maxH = localPoints.Max(p => p.H);

    // Ensure we start at ~0 (just above plane) for sanity
    if (minH < 0) minH = 0;

    int sliceCount = (int)Math.Ceiling((maxH - minH) / sliceThickness);
    if (sliceCount <= 0) return 0.0;

    double totalVolume = 0.0;

    // ---- 4. Slice loop ----
    for (int i = 0; i < sliceCount; i++)
    {
        double h0 = minH + i * sliceThickness;
        double h1 = h0 + sliceThickness;
        double hCenter = 0.5 * (h0 + h1);

        // Points belonging to this slice
        var slicePoints = localPoints
            .Where(p => p.H >= h0 && p.H < h1)
            .ToList();

        if (slicePoints.Count < minPointsPerSlice)
            continue;

        // Compute centroid in local (u,v) for this slice
        double cx = slicePoints.Average(p => p.U);
        double cy = slicePoints.Average(p => p.V);

        // Compute RMS radius around that centroid
        double avgR2 = slicePoints.Average(p =>
        {
            double du = p.U - cx;
            double dv = p.V - cy;
            return du * du + dv * dv;
        });

        double radius = Math.Sqrt(avgR2);
        if (radius <= 0) continue;

        double area = Math.PI * radius * radius;
        double volumeSlice = area * sliceThickness;
        totalVolume += volumeSlice;

        // World-space center of this slice
        Point3D centerWorld =
            origin +
            n * hCenter +
            u * cx +
            v * cy;

        slices.Add(new BlobSlice
        {
            H0          = h0,
            H1          = h1,
            HCenter     = hCenter,
            CenterWorld = centerWorld,
            Normal      = n,
            Radius      = radius,
            Area        = area,
            PointCount  = slicePoints.Count
        });
    }

    return totalVolume;
}