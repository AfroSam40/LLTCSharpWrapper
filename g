using System;
using System.Collections.Generic;
using System.Linq;
using System.Windows.Media.Media3D;

namespace PointCloudUtils
{
    public class PlaneFitResult
    {
        /// <summary>Plane coefficients: z = A*x + B*y + C</summary>
        public double A { get; set; }
        public double B { get; set; }
        public double C { get; set; }

        /// <summary>Plane normal (normalized).</summary>
        public Vector3D Normal { get; set; }

        /// <summary>Centroid of the inlier points used for the fit.</summary>
        public Point3D Centroid { get; set; }

        /// <summary>Average absolute distance of inlier points to plane.</summary>
        public double AverageError { get; set; }

        /// <summary>Root-mean-square distance of inlier points to plane.</summary>
        public double Rmse { get; set; }

        /// <summary>Peak-to-valley flatness (max distance - min distance).</summary>
        public double Flatness { get; set; }

        /// <summary>Standard deviation of signed distances to plane.</summary>
        public double StdDev { get; set; }

        /// <summary>The points that belong to this plane (height band).</summary>
        public List<Point3D> InlierPoints { get; set; } = new List<Point3D>();
    }

    public static class PlaneMetrics
    {
        /// <summary>
        /// Computes flatness-related metrics for a fitted plane:
        /// - Flatness (peak-to-valley)
        /// - AverageError (mean absolute distance)
        /// - Rmse (root mean square distance)
        /// - StdDev (standard deviation of signed distances)
        /// 
        /// Uses orthogonal distance along plane.Normal.
        /// </summary>
        public static void ComputeFlatness(PlaneFitResult plane)
        {
            if (plane == null)
                throw new ArgumentNullException(nameof(plane));

            var pts = plane.InlierPoints;
            if (pts == null || pts.Count == 0)
            {
                plane.Flatness = 0;
                plane.AverageError = 0;
                plane.Rmse = 0;
                plane.StdDev = 0;
                return;
            }

            // Normal must be normalized for distances to be in correct units.
            Vector3D n = plane.Normal;
            if (n.LengthSquared < 1e-12)
                throw new InvalidOperationException("Plane normal is zero-length.");

            n.Normalize();

            int nPts = pts.Count;

            double sumAbs = 0.0;
            double sumSq = 0.0;
            double sum = 0.0;

            double minD = double.MaxValue;
            double maxD = double.MinValue;

            // First pass: compute signed distances and basic stats
            var distances = new double[nPts];

            for (int i = 0; i < nPts; i++)
            {
                var p = pts[i];

                // Signed orthogonal distance from point to plane through Centroid with normal n
                // d = n · (p - centroid)
                var v = p - plane.Centroid;
                double d = n.X * v.X + n.Y * v.Y + n.Z * v.Z;
                distances[i] = d;

                double abs = Math.Abs(d);
                sumAbs += abs;
                sumSq += d * d;
                sum += d;

                if (d < minD) minD = d;
                if (d > maxD) maxD = d;
            }

            double mean = sum / nPts;

            // Second pass: compute variance for standard deviation
            double varSum = 0.0;
            for (int i = 0; i < nPts; i++)
            {
                double diff = distances[i] - mean;
                varSum += diff * diff;
            }

            // You can use nPts for population std-dev, or (nPts - 1) for sample std-dev.
            // Here we use population:
            double variance = (nPts > 0) ? varSum / nPts : 0.0;
            double stdDev = Math.Sqrt(variance);

            plane.AverageError = sumAbs / nPts;          // mean |d|
            plane.Rmse        = Math.Sqrt(sumSq / nPts); // sqrt(mean d²)
            plane.Flatness    = maxD - minD;             // peak-to-valley
            plane.StdDev      = stdDev;                  // σ of signed distances
        }
    }
}