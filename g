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
        /// The points that belong to this plane (height band).
        /// </summary>
        public List<Point3D> InlierPoints { get; set; } = new List<Point3D>();
    }

    public static class PointCloudPlaneFitting
    {
        /// <summary>
        /// Finds multiple best-fit planes in a point cloud, grouped by height.
        /// Points are sorted by Z, grouped into bands of thickness bandThickness,
        /// a full least-squares plane z = A x + B y + C is fit to each band,
        /// and only planes whose normal is within maxTiltDegrees of the Z-axis
        /// are kept.
        /// </summary>
        /// <param name="points">Input point cloud.</param>
        /// <param name="bandThickness">Max Z-span per band.</param>
        /// <param name="minPointsPerPlane">Minimum number of points required to fit a plane.</param>
        /// <param name="maxTiltDegrees">
        /// Maximum allowed tilt (in degrees) between plane normal and global Z-axis.
        /// e.g. 5–10 degrees.
        /// </param>
        public static List<PlaneFitResult> FitHorizontalPlanesByHeight(
            Point3DCollection points,
            double bandThickness,
            int minPointsPerPlane = 100,
            double maxTiltDegrees = 10.0)
        {
            var results = new List<PlaneFitResult>();
            if (points == null || points.Count == 0)
                return results;

            // 1. Sort points by Z
            var sorted = points.OrderBy(p => p.Z).ToList();

            // 2. Group into bands along Z
            var currentBand = new List<Point3D>();
            double currentBandStartZ = sorted[0].Z;

            foreach (var p in sorted)
            {
                if (currentBand.Count == 0)
                {
                    currentBand.Add(p);
                    currentBandStartZ = p.Z;
                    continue;
                }

                if (Math.Abs(p.Z - currentBandStartZ) <= bandThickness)
                {
                    currentBand.Add(p);
                }
                else
                {
                    // Finish current band
                    TryFitAndAddPlane(currentBand, minPointsPerPlane, maxTiltDegrees, results);

                    // Start new band
                    currentBand = new List<Point3D> { p };
                    currentBandStartZ = p.Z;
                }
            }

            // Last band
            TryFitAndAddPlane(currentBand, minPointsPerPlane, maxTiltDegrees, results);

            return results;
        }

        private static void TryFitAndAddPlane(
            List<Point3D> band,
            int minPointsPerPlane,
            double maxTiltDegrees,
            List<PlaneFitResult> results)
        {
            if (band.Count < minPointsPerPlane)
                return;

            var plane = FitHorizontalPlane(band);
            if (plane == null)
                return;

            // Angle between plane normal and global Z axis
            Vector3D zAxis = new Vector3D(0, 0, 1);
            double dot = Math.Abs(Vector3D.DotProduct(plane.Normal, zAxis)); // use abs so +/-Z both accepted
            dot = Math.Max(-1.0, Math.Min(1.0, dot)); // clamp numeric noise
            double angleDeg = Math.Acos(dot) * 180.0 / Math.PI;

            if (angleDeg <= maxTiltDegrees)
            {
                results.Add(plane);
            }
            // else: plane is too tilted, ignore this band
        }

        /// <summary>
        /// Fits a single plane z = a*x + b*y + c (least squares)
        /// to the given surface points.
        /// Returns null if the system is degenerate.
        /// </summary>
        private static PlaneFitResult? FitHorizontalPlane(List<Point3D> pts)
        {
            int n = pts.Count;
            if (n < 3)
                return null;

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

            double a11 = sumX2, a12 = sumXY, a13 = sumX;
            double a21 = sumXY, a22 = sumY2, a23 = sumY;
            double a31 = sumX,  a32 = sumY,  a33 = n;

            double b1 = sumXZ, b2 = sumYZ, b3 = sumZ;

            double detA =
                a11 * (a22 * a33 - a23 * a32) -
                a12 * (a21 * a33 - a23 * a31) +
                a13 * (a21 * a32 - a22 * a31);

            if (Math.Abs(detA) < 1e-12)
                return null;

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

            var centroid = new Point3D(sumX / n, sumY / n, sumZ / n);

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