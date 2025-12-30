using System;
using System.Linq;
using System.Windows.Media.Media3D;

public static class PlaneMetrics
{
    /// <summary>
    /// Compute GD&T-style flatness and some error stats from a PlaneFitResult.
    /// 
    /// Flatness here = max(normal-distance) - min(normal-distance)
    /// over all inlier points.
    /// </summary>
    /// <param name="plane">PlaneFitResult with Normal, Centroid, and InlierPoints populated.</param>
    /// <param name="flatness">
    /// Peak-to-valley distance (same units as your point cloud, e.g. mm).
    /// This is the "equivalent flatness".
    /// </param>
    /// <param name="avgAbsError">
    /// Mean absolute distance to plane (along the normal). Often close to your AverageError.
    /// </param>
    /// <param name="rmsError">
    /// Root-mean-square distance to plane (along the normal).
    /// </param>
    public static void ComputeFlatness(
        PlaneFitResult plane,
        out double flatness,
        out double avgAbsError,
        out double rmsError)
    {
        flatness = 0;
        avgAbsError = 0;
        rmsError = 0;

        if (plane == null ||
            plane.InlierPoints == null ||
            plane.InlierPoints.Count < 3)
        {
            return;
        }

        // Make sure normal is normalized
        Vector3D n = plane.Normal;
        if (n.LengthSquared == 0)
            throw new InvalidOperationException("Plane normal has zero length.");

        n.Normalize();

        var pts = plane.InlierPoints;
        int nPts = pts.Count;

        double minD = double.PositiveInfinity;
        double maxD = double.NegativeInfinity;
        double sumAbs = 0.0;
        double sumSq = 0.0;

        // Use centroid as a reference point on the plane
        Point3D p0 = plane.Centroid;

        foreach (var p in pts)
        {
            // Vector from reference point on plane to point
            var v = new Vector3D(p.X - p0.X, p.Y - p0.Y, p.Z - p0.Z);

            // Signed distance along the plane normal (can be + or -)
            double d = Vector3D.DotProduct(v, n);

            if (d < minD) minD = d;
            if (d > maxD) maxD = d;

            double abs = Math.Abs(d);
            sumAbs += abs;
            sumSq  += d * d;
        }

        flatness     = maxD - minD;                // peak-to-valley
        avgAbsError  = sumAbs / nPts;              // mean |distance|
        rmsError     = Math.Sqrt(sumSq / nPts);    // RMS distance
    }
}