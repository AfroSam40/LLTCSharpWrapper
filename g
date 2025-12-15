public static double EstimateBlobVolumeBySlicesUsingHull(
    Point3DCollection points,
    PlaneFitResult basePlane,
    double sliceThickness,
    int minPointsPerSlice,
    out List<BlobSlice> slices)
{
    slices = new List<BlobSlice>();

    if (points == null || points.Count == 0)
        return 0.0;
    if (sliceThickness <= 0.0)
        throw new ArgumentOutOfRangeException(nameof(sliceThickness));

    // ---------- 1. Orthonormal basis (u, v, n) with AUTO NORMAL ORIENTATION ----------
    Vector3D n = basePlane.Normal;
    if (n.LengthSquared < 1e-12)
        throw new ArgumentException("Base plane normal is zero.", nameof(basePlane));

    n.Normalize();

    // Flip n so that most points lie at positive height
    double signSum = 0.0;
    foreach (var p in points)
    {
        Vector3D d = p - basePlane.Centroid;
        double h = Vector3D.DotProduct(d, n);
        if (h > 0) signSum += 1.0;
        else if (h < 0) signSum -= 1.0;
    }
    if (signSum < 0.0)
        n = -n;

    // Build tangential basis u, v
    Vector3D temp = Math.Abs(n.Z) < 0.9
        ? new Vector3D(0, 0, 1)
        : new Vector3D(1, 0, 0);

    Vector3D u = Vector3D.CrossProduct(temp, n);
    if (u.LengthSquared < 1e-12)
        u = new Vector3D(1, 0, 0);
    u.Normalize();

    Vector3D v = Vector3D.CrossProduct(n, u);
    v.Normalize();

    Point3D origin = basePlane.Centroid;

    // ---------- 2. Transform all points to local (U, V, H) ----------
    var local = new List<(double U, double V, double H)>(points.Count);
    double sumUAll = 0.0, sumVAll = 0.0;

    foreach (var p in points)
    {
        Vector3D d = p - origin;
        double h = Vector3D.DotProduct(d, n); // height above plane

        if (h < -1e-6)
            continue; // clearly below plane (ignore)

        double uu = Vector3D.DotProduct(d, u);
        double vv = Vector3D.DotProduct(d, v);

        local.Add((uu, vv, h));
        sumUAll += uu;
        sumVAll += vv;
    }

    if (local.Count == 0)
        return 0.0;

    // Global in-plane centroid for the whole blob
    double globalU = sumUAll / local.Count;
    double globalV = sumVAll / local.Count;

    double minH = local.Min(t => t.H);
    double maxH = local.Max(t => t.H);
    if (minH < 0) minH = 0;

    int sliceCount = (int)Math.Ceiling((maxH - minH) / sliceThickness);
    if (sliceCount <= 0)
        return 0.0;

    var sliceBuckets = new List<List<(double U, double V, double H)>>(sliceCount);
    for (int i = 0; i < sliceCount; i++)
        sliceBuckets.Add(new List<(double U, double V, double H)>());

    foreach (var (U, V, H) in local)
    {
        int idx = (int)((H - minH) / sliceThickness);
        if (idx < 0) idx = 0;
        if (idx >= sliceCount) idx = sliceCount - 1;
        sliceBuckets[idx].Add((U, V, H));
    }

    // ---------- 3. Slice hull area, global center for visualization ----------
    double totalVolume = 0.0;

    for (int i = 0; i < sliceCount; i++)
    {
        var bucket = sliceBuckets[i];
        if (bucket.Count < minPointsPerSlice)
            continue;

        // 2D points for convex hull (area only)
        var pts2D = new List<Point>(bucket.Count);
        foreach (var (U, V, _) in bucket)
            pts2D.Add(new Point(U, V));

        double area = ComputeConvexHullArea(pts2D);
        if (area <= 0)
            continue;

        double h0 = minH + i * sliceThickness;
        double h1 = h0 + sliceThickness;
        double hCenter = 0.5 * (h0 + h1);

        // Center for visualization uses GLOBAL (U,V),
        // so slices stack nicely along a single axis
        Point3D centerWorld =
            origin +
            u * globalU +
            v * globalV +
            n * hCenter;

        double radius = Math.Sqrt(area / Math.PI);

        slices.Add(new BlobSlice
        {
            H0 = h0,
            H1 = h1,
            HCenter = hCenter,
            // store both for debugging
            UCenter = globalU,
            VCenter = globalV,
            CenterWorld = centerWorld,
            Normal = n,
            Radius = radius,
            Area = area,
            PointCount = bucket.Count
        });

        totalVolume += area * sliceThickness;
    }

    return totalVolume;
}