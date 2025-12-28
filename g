public static List<PlaneFitResult> FitHorizontalPlanesByHeight(
    Point3DCollection points,
    double bandThickness,
    int minPointsPerPlane = 100)
{
    var results = new List<PlaneFitResult>();
    if (points == null || points.Count == 0)
        return results;

    // Sort points by Z (ascending)
    var sorted = points.OrderBy(p => p.Z).ToList();

    // Current surface accumulator
    var currentSurface = new List<Point3D>();
    double currentMeanZ = sorted[0].Z;

    currentSurface.Add(sorted[0]);

    for (int i = 1; i < sorted.Count; i++)
    {
        var p = sorted[i];

        // How far is this point from the current surface mean height?
        double dzToMean = Math.Abs(p.Z - currentMeanZ);

        if (dzToMean <= bandThickness)
        {
            // Still the same surface – add and update running mean
            currentSurface.Add(p);
            int n = currentSurface.Count;
            currentMeanZ += (p.Z - currentMeanZ) / n;   // incremental mean update
        }
        else
        {
            // We hit a "new" surface in Z -> finish the current one
            if (currentSurface.Count >= minPointsPerPlane)
            {
                var plane = FitHorizontalPlane(currentSurface);
                if (plane != null)
                    results.Add(plane);
            }

            // Start a fresh surface with this point
            currentSurface = new List<Point3D> { p };
            currentMeanZ = p.Z;
        }
    }

    // Final surface
    if (currentSurface.Count >= minPointsPerPlane)
    {
        var plane = FitHorizontalPlane(currentSurface);
        if (plane != null)
            results.Add(plane);
    }

    return results;
}

private static PlaneFitResult? FitHorizontalPlane(List<Point3D> pts)
{
    int n = pts.Count;
    if (n < 3)
        return null;

    double sumX = 0, sumY = 0, sumZ = 0;
    foreach (var p in pts)
    {
        sumX += p.X;
        sumY += p.Y;
        sumZ += p.Z;
    }

    double cx = sumX / n;
    double cy = sumY / n;
    double cz = sumZ / n;

    // Force plane to be z = C (parallel to XY)
    double A = 0.0;
    double B = 0.0;
    double C = cz;

    var normal = new Vector3D(0, 0, 1); // strictly vertical

    double errSum = 0.0;
    foreach (var p in pts)
        errSum += Math.Abs(p.Z - C);

    double avgErr = errSum / n;

    return new PlaneFitResult
    {
        A = A,
        B = B,
        C = C,
        Normal = normal,
        Centroid = new Point3D(cx, cy, cz),
        AverageError = avgErr,
        InlierPoints = new List<Point3D>(pts)
    };
}