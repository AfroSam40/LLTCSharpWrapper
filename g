public static double EstimateBlobVolumeBySlicesUsingHull_Cumulative(
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

    // ---------- 1. Orthonormal basis (u, v, n) with auto normal orientation ----------
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

    // Tangent basis in plane
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

        // keep only points at / above the plane (small tolerance)
        if (h < -1e-6)
            continue;

        double uu = Vector3D.DotProduct(d, u);
        double vv = Vector3D.DotProduct(d, v);

        local.Add((uu, vv, h));
        sumUAll += uu;
        sumVAll += vv;
    }

    if (local.Count == 0)
        return 0.0;

    // One global in-plane centroid (for nice visual alignment)
    double globalU = sumUAll / local.Count;
    double globalV = sumVAll / local.Count;

    double minH = local.Min(t => t.H);
    double maxH = local.Max(t => t.H);
    if (minH < 0) minH = 0;

    int sliceCount = (int)Math.Ceiling((maxH - minH) / sliceThickness);
    if (sliceCount <= 0)
        return 0.0;

    // ---------- 3. Cumulative hull per slice ----------
    double totalVolume = 0.0;
    double prevCumArea = 0.0;

    for (int i = 0; i < sliceCount; i++)
    {
        double hTop = minH + (i + 1) * sliceThickness;
        double hBottom = minH + i * sliceThickness;
        double hCenter = 0.5 * (hBottom + hTop);

        // cumulative: all points from base (minH) up to this slice top
        var bucket = new List<(double U, double V, double H)>();
        foreach (var t in local)
        {
            if (t.H >= minH && t.H <= hTop)
                bucket.Add(t);
        }

        if (bucket.Count < minPointsPerSlice)
            continue;

        var pts2D = new List<Point>(bucket.Count);
        foreach (var (U, V, _) in bucket)
            pts2D.Add(new Point(U, V));

        // CUMULATIVE hull area at this height
        double cumArea = ComputeConvexHullArea(pts2D);
        if (cumArea <= 0)
            continue;

        // effective cross-section area for this band = area difference
        double bandArea = cumArea - prevCumArea;
        if (bandArea < 0) bandArea = 0; // guard against numerical noise

        // physical volume contribution for this slice
        totalVolume += bandArea * sliceThickness;

        // Visual center: use global (U, V) so slices line up smoothly
        Point3D centerWorld =
            origin +
            u * globalU +
            v * globalV +
            n * hCenter;

        double radius = bandArea > 0
            ? Math.Sqrt(bandArea / Math.PI)
            : Math.Sqrt(cumArea / Math.PI); // fallback

        slices.Add(new BlobSlice
        {
            H0 = hBottom,
            H1 = hTop,
            HCenter = hCenter,
            UCenter = globalU,
            VCenter = globalV,
            CenterWorld = centerWorld,
            Normal = n,
            Radius = radius,
            Area = bandArea,      // band area (effective cross-section)
            PointCount = bucket.Count
        });

        prevCumArea = cumArea;
    }

    return totalVolume;
}