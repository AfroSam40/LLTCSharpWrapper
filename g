using System;
using System.Collections.Generic;
using System.Linq;
using System.Windows.Media.Media3D;

namespace PointCloudUtils
{
    /// <summary>
    /// Result of fitting a (roughly horizontal) plane z = a*x + b*y + c.
    /// </summary>
    public class PlaneFitResult
    {
        /// <summary>
        /// Plane coefficients: z = A * x + B * y + C
        /// </summary>
        public double A { get; set; }
        public double B { get; set; }
        public double C { get; set; }

        /// <summary>
        /// Plane normal (normalized).
        /// </summary>
        public Vector3D Normal { get; set; }

        /// <summary>
        /// Centroid of the inlier points used for the fit.
        /// </summary>
        public Point3D Centroid { get; set; }

        /// <summary>
        /// Average absolute distance (in Z) from points to plane.
        /// </summary>
        public double AverageError { get; set; }

        /// <summary>
        /// The points that belong to this plane (height band / surface).
        /// </summary>
        public List<Point3D> InlierPoints { get; set; } = new List<Point3D>();
    }

    public static class PointCloudPlaneFitting
    {
        /// <summary>
        /// Finds multiple best-fit (roughly horizontal) planes in a point cloud.
        /// 
        /// Points are first sorted by Z. We then walk that list and build
        /// contiguous "surfaces" such that consecutive points whose Z
        /// differs by at most <paramref name="bandThickness"/> belong to
        /// the same surface. A "significant" jump in Z (greater than this
        /// threshold) starts a new surface.
        /// 
        /// So:
        ///   - bandThickness  = "significant height change" threshold
        ///   - minPointsPerPlane = only used to discard tiny/noisy surfaces
        /// 
        /// Assumes surfaces are mostly parallel to the XY plane.
        /// </summary>
        /// <param name="points">Input point cloud.</param>
        /// <param name="bandThickness">
        /// Maximum allowed |ΔZ| between *consecutive* sorted points to still
        /// be considered the same surface. If the jump in Z between neighbors
        /// is larger than this, we treat that as a transition to another surface.
        /// </param>
        /// <param name="minPointsPerPlane">
        /// Minimum number of points a surface must have to be accepted.
        /// Small clusters below this are ignored as noise, but they do NOT
        /// affect where surface boundaries are placed.
        /// </param>
        /// <returns>List of plane-fit results (one per detected surface).</returns>
        public static List<PlaneFitResult> FitHorizontalPlanesByHeight(
            Point3DCollection points,
            double bandThickness,
            int minPointsPerPlane = 100)
        {
            var results = new List<PlaneFitResult>();
            if (points == null || points.Count == 0)
                return results;

            // 1. Sort points by Z (ascending)
            var sorted = points.OrderBy(p => p.Z).ToList();

            // 2. Build surfaces based on *consecutive* Z jumps
            var currentSurface = new List<Point3D>();
            currentSurface.Add(sorted[0]);
            double lastZ = sorted[0].Z;

            for (int i = 1; i < sorted.Count; i++)
            {
                var p = sorted[i];
                double dz = Math.Abs(p.Z - lastZ);

                if (dz <= bandThickness)
                {
                    // Still the same surface
                    currentSurface.Add(p);
                }
                else
                {
                    // We hit a significant jump in Z -> finish current surface
                    if (currentSurface.Count >= minPointsPerPlane)
                    {
                        var plane = FitHorizontalPlane(currentSurface);
                        if (plane != null)
                            results.Add(plane);
                    }

                    // Start a new surface
                    currentSurface = new List<Point3D> { p };
                }

                lastZ = p.Z;
            }

            // 3. Final surface
            if (currentSurface.Count >= minPointsPerPlane)
            {
                var plane = FitHorizontalPlane(currentSurface);
                if (plane != null)
                    results.Add(plane);
            }

            return results;
        }

        /// <summary>
        /// Fits a single plane z = a*x + b*y + c (least squares)
        /// to the given (roughly horizontal) surface points.
        /// Returns null if the system is degenerate.
        /// </summary>
        private static PlaneFitResult? FitHorizontalPlane(List<Point3D> pts)
        {
            int n = pts.Count;
            if (n < 3)
                return null;

            // Accumulate sums for normal equations
            double sumX = 0, sumY = 0, sumZ = 0;
            double sumX2 = 0, sumY2 = 0, sumXY = 0;
            double sumXZ = 0, sumYZ = 0;

            foreach (var p in pts)
            {
                double x = p.X;
                double y = p.Y;
                double z = p.Z;

                sumX  += x;
                sumY  += y;
                sumZ  += z;
                sumX2 += x * x;
                sumY2 += y * y;
                sumXY += x * y;
                sumXZ += x * z;
                sumYZ += y * z;
            }

            // Normal equation matrix A and RHS b for z = a*x + b*y + c:
            // [ sumX2  sumXY  sumX ] [a] = [ sumXZ ]
            // [ sumXY  sumY2  sumY ] [b]   [ sumYZ ]
            // [ sumX   sumY   n    ] [c]   [ sumZ  ]
            double a11 = sumX2, a12 = sumXY, a13 = sumX;
            double a21 = sumXY, a22 = sumY2, a23 = sumY;
            double a31 = sumX,  a32 = sumY,  a33 = n;

            double b1 = sumXZ, b2 = sumYZ, b3 = sumZ;

            double detA =
                a11 * (a22 * a33 - a23 * a32) -
                a12 * (a21 * a33 - a23 * a31) +
                a13 * (a21 * a32 - a22 * a31);

            if (Math.Abs(detA) < 1e-12)
            {
                // Degenerate system – points may be collinear or too noisy
                return null;
            }

            double detA1 =
                b1  * (a22 * a33 - a23 * a32) -
                a12 * (b2  * a33 - a23 * b3 ) +
                a13 * (b2  * a32 - a22 * b3 );

            double detA2 =
                a11 * (b2  * a33 - a23 * b3 ) -
                b1  * (a21 * a33 - a23 * a31) +
                a13 * (a21 * b3  - b2  * a31);

            double detA3 =
                a11 * (a22 * b3  - b2  * a32) -
                a12 * (a21 * b3  - b2  * a31) +
                b1  * (a21 * a32 - a22 * a31);

            double A = detA1 / detA;
            double B = detA2 / detA;
            double C = detA3 / detA;

            // Normal of plane z - A*x - B*y - C = 0 is (A, B, -1)
            Vector3D normal = new Vector3D(A, B, -1.0);
            if (normal.Length > 0)
                normal.Normalize();

            // Centroid
            var centroid = new Point3D(sumX / n, sumY / n, sumZ / n);

            // Average absolute error in Z
            double errSum = 0;
            foreach (var p in pts)
            {
                double zFit = A * p.X + B * p.Y + C;
                errSum += Math.Abs(p.Z - zFit);
            }
            double avgErr = errSum / n;

            return new PlaneFitResult
            {
                A = A,
                B = B,
                C = C,
                Normal = normal,
                Centroid = centroid,
                AverageError = avgErr,
                InlierPoints = new List<Point3D>(pts)
            };
        }
    }
}