public double A { get; set; }    // z = A*x + B*y + C
public double B { get; set; }
public double C { get; set; }
public Vector3D Normal { get; set; }
public Point3D  Centroid { get; set; }
public double   AverageError { get; set; }   // mean |residual|
public double   Rmse { get; set; }           // sqrt(mean residual^2)
public List<Point3D> InlierPoints { get; set; } = new();

-----

public static List<PlaneFitResult> FitHorizontalPlanesByHeight(
    Point3DCollection points,
    double bandThickness,
    int minPointsPerPlane = 100)
{
    var results = new List<PlaneFitResult>();
    if (points == null || points.Count == 0) return results;

    // 1) sort by Z
    var sorted = points.OrderBy(p => p.Z).ToList();

    // 2) sweep in Z and form bands whose Z-span <= bandThickness
    var currentBand = new List<Point3D>();
    double currentBandStartZ = sorted[0].Z;

    void FlushBandIfAny()
    {
        if (currentBand.Count >= minPointsPerPlane)
        {
            var fit = FitPlaneLeastSquares(currentBand);
            results.Add(fit);
        }
        currentBand.Clear();
    }

    foreach (var p in sorted)
    {
        if (currentBand.Count == 0)
        {
            currentBand.Add(p);
            currentBandStartZ = p.Z;
            continue;
        }

        // still within the band?
        if (p.Z - currentBandStartZ <= bandThickness)
        {
            currentBand.Add(p);
        }
        else
        {
            // close current band, start a new one
            FlushBandIfAny();
            currentBand.Add(p);
            currentBandStartZ = p.Z;
        }
    }

    // last band
    FlushBandIfAny();

    return results;
}

// --- helpers ---

private static PlaneFitResult FitPlaneLeastSquares(List<Point3D> pts)
{
    // Fit z = A*x + B*y + C via normal equations
    // Build sums
    double Sx=0, Sy=0, Sz=0, Sxx=0, Syy=0, Sxy=0, Sxz=0, Syz=0;
    int n = pts.Count;

    foreach (var p in pts)
    {
        double x = p.X, y = p.Y, z = p.Z;
        Sx  += x;      Sy  += y;      Sz  += z;
        Sxx += x*x;    Syy += y*y;    Sxy += x*y;
        Sxz += x*z;    Syz += y*z;
    }

    // Solve:
    // [Sxx Sxy Sx][A] = [Sxz]
    // [Sxy Syy Sy][B]   [Syz]
    // [Sx  Sy  n ][C]   [Sz ]
    double[,] M = {
        { Sxx, Sxy, Sx },
        { Sxy, Syy, Sy },
        { Sx , Sy , n  }
    };
    double[] b = { Sxz, Syz, Sz };

    var abc = Solve3x3(M, b); // returns length-3 array
    double A = abc[0], B = abc[1], C = abc[2];

    // Centroid
    var centroid = new Point3D(Sx / n, Sy / n, Sz / n);

    // Plane normal from z = A x + B y + C  →  Ax + By - z + C = 0 ⇒ normal (A, B, -1)
    var normal = new Vector3D(A, B, -1);
    normal.Normalize();

    // Residuals: ei = (A*xi + B*yi + C - zi)
    double absSum = 0.0;
    double sqSum  = 0.0;
    foreach (var p in pts)
    {
        double e = (A * p.X + B * p.Y + C) - p.Z;
        absSum += Math.Abs(e);
        sqSum  += e * e;
    }

    var result = new PlaneFitResult
    {
        A = A,
        B = B,
        C = C,
        Normal = normal,
        Centroid = centroid,
        AverageError = absSum / n,           // mean absolute error
        Rmse = Math.Sqrt(sqSum / n),         // root mean square error
        InlierPoints = new List<Point3D>(pts)
    };

    return result;
}

private static double[] Solve3x3(double[,] M, double[] b)
{
    // Cramer's rule / adjoint-based solve for small 3x3 (no external deps)
    double a11 = M[0,0], a12 = M[0,1], a13 = M[0,2];
    double a21 = M[1,0], a22 = M[1,1], a23 = M[1,2];
    double a31 = M[2,0], a32 = M[2,1], a33 = M[2,2];

    double det =
        a11*(a22*a33 - a23*a32) -
        a12*(a21*a33 - a23*a31) +
        a13*(a21*a32 - a22*a31);

    if (Math.Abs(det) < 1e-12)
        throw new InvalidOperationException("Singular matrix in plane fit.");

    // inverse(M) * b
    double inv11 =  (a22*a33 - a23*a32) / det;
    double inv12 = -(a12*a33 - a13*a32) / det;
    double inv13 =  (a12*a23 - a13*a22) / det;

    double inv21 = -(a21*a33 - a23*a31) / det;
    double inv22 =  (a11*a33 - a13*a31) / det;
    double inv23 = -(a11*a23 - a13*a21) / det;

    double inv31 =  (a21*a32 - a22*a31) / det;
    double inv32 = -(a11*a32 - a12*a31) / det;
    double inv33 =  (a11*a22 - a12*a21) / det;

    return new[]
    {
        inv11*b[0] + inv12*b[1] + inv13*b[2],
        inv21*b[0] + inv22*b[1] + inv23*b[2],
        inv31*b[0] + inv32*b[1] + inv33*b[2]
    };
}