using System;
using System.Collections.Generic;
using System.Linq;
using System.Windows;
using System.Windows.Media.Media3D;

namespace PointCloudUtils
{
    public class BlobSlice
    {
        public double H0 { get; set; }      // bottom of slice (height above base plane)
        public double H1 { get; set; }      // top of slice
        public double Area { get; set; }    // convex hull area for this slice
        public double Volume { get; set; }  // Area * (H1 - H0)
        public List<Point> Hull2D { get; set; } = new List<Point>();
    }

    public static class BlobVolumeTools
    {
        /// <summary>
        /// Estimates volume of a "blob" sitting on basePlane by slicing in height
        /// and using 2D convex hull per slice. Now with 2D outlier rejection
        /// before hull.
        /// </summary>
        /// <param name="points">3D point cloud of the blob + base around it.</param>
        /// <param name="basePlane">Best-fit plane the blob sits on.</param>
        /// <param name="sliceThickness">Height step in same units as points (e.g. mm).</param>
        /// <param name="slices">Per-slice info (heights, area, hull, volume).</param>
        /// <param name="minPointsPerSlice">
        /// Minimum inlier points required to keep a slice. Default 50.
        /// </param>
        /// <param name="outlierSigma">
        /// Radial sigma threshold for outlier removal in 2D (e.g. 2.5–3.0). 
        /// Set &lt;= 0 to disable outlier filtering.
        /// </param>
        public static double EstimateBlobVolumeByHullSlices(
            Point3DCollection points,
            PlaneFitResult basePlane,
            double sliceThickness,
            out List<BlobSlice> slices,
            int minPointsPerSlice = 50,
            double outlierSigma = 2.5)
        {
            slices = new List<BlobSlice>();
            if (points == null || points.Count == 0 || sliceThickness <= 0)
                return 0.0;

            // 1) Define plane normal and local 2D basis (u, v) on the plane
            Vector3D n = basePlane.Normal;
            if (n.Length < 1e-9)
            {
                // Fallback if Normal not set
                n = new Vector3D(basePlane.A, basePlane.B, -1.0);
            }
            n.Normalize();

            // You already have this pattern elsewhere:
            Vector3D arbitrary = Math.Abs(n.Z) < 0.9
                ? new Vector3D(0, 0, 1)
                : new Vector3D(0, 1, 0);

            Vector3D u = Vector3D.CrossProduct(n, arbitrary);
            u.Normalize();
            Vector3D v = Vector3D.CrossProduct(n, u);
            v.Normalize();

            var origin = basePlane.Centroid;

            // 2) Project all points once: we store (x2d, y2d, h)
            var proj = new List<(double x, double y, double h)>(points.Count);
            double maxH = double.MinValue;

            foreach (var p in points)
            {
                // Vertical height above plane using plane equation
                double zPlane = basePlane.A * p.X + basePlane.B * p.Y + basePlane.C;
                double h = p.Z - zPlane;  // > 0 means above base plane

                if (h <= 0.0)
                    continue;             // ignore points below/at the base

                // 2D coordinates in plane's local frame
                Vector3D d = p - origin;
                double x2d = Vector3D.DotProduct(d, u);
                double y2d = Vector3D.DotProduct(d, v);

                proj.Add((x2d, y2d, h));
                if (h > maxH) maxH = h;
            }

            if (proj.Count == 0 || maxH <= 0)
                return 0.0;

            double totalVolume = 0.0;

            // 3) Slice in height [0, maxH)
            for (double h0 = 0.0; h0 < maxH; h0 += sliceThickness)
            {
                double h1 = h0 + sliceThickness;

                // collect 2D points whose height is in this band
                var slice2D = new List<Point>();
                foreach (var (x, y, h) in proj)
                {
                    if (h >= h0 && h < h1)
                        slice2D.Add(new Point(x, y));
                }

                if (slice2D.Count < minPointsPerSlice)
                    continue;

                // --- OUTLIER FILTERING (2D radial from centroid) ---
                var inliers2D = RemoveRadialOutliers(slice2D, outlierSigma);

                if (inliers2D.Count < 3)  // not enough to form a hull
                    continue;

                // 4) Convex hull on inlier points only
                var hull = ComputeConvexHull2D(inliers2D);
                if (hull == null || hull.Count < 3)
                    continue;

                double area = PolygonArea(hull);
                if (area <= 0)
                    continue;

                double sliceVolume = area * sliceThickness;
                totalVolume += sliceVolume;

                slices.Add(new BlobSlice
                {
                    H0 = h0,
                    H1 = h1,
                    Area = area,
                    Volume = sliceVolume,
                    Hull2D = hull
                });
            }

            return totalVolume;
        }

        /// <summary>
        /// Remove 2D outliers based on radial distance from centroid.
        /// Keeps points with distance <= mean + sigma * stddev.
        /// If sigma &lt;= 0 or the filter removes too many points, returns original.
        /// </summary>
        private static List<Point> RemoveRadialOutliers(IList<Point> pts, double sigma)
        {
            int n = pts.Count;
            if (n <= 3 || sigma <= 0)
                return new List<Point>(pts);

            // centroid
            double sumX = 0, sumY = 0;
            for (int i = 0; i < n; i++)
            {
                sumX += pts[i].X;
                sumY += pts[i].Y;
            }
            double cx = sumX / n;
            double cy = sumY / n;

            // distances
            double[] r = new double[n];
            double sumR = 0, sumR2 = 0;
            for (int i = 0; i < n; i++)
            {
                double dx = pts[i].X - cx;
                double dy = pts[i].Y - cy;
                double d = Math.Sqrt(dx * dx + dy * dy);
                r[i] = d;
                sumR += d;
                sumR2 += d * d;
            }

            double mean = sumR / n;
            double var = Math.Max(sumR2 / n - mean * mean, 0.0);
            double std = Math.Sqrt(var);

            if (std <= 0)
                return new List<Point>(pts);  // all distances almost identical

            double maxR = mean + sigma * std;

            var inliers = new List<Point>(n);
            for (int i = 0; i < n; i++)
            {
                if (r[i] <= maxR)
                    inliers.Add(pts[i]);
            }

            // If filter nuked almost everything, fall back to original
            if (inliers.Count < 3)
                return new List<Point>(pts);

            return inliers;
        }

        /// <summary>
        /// Standard 2D convex hull (monotone chain).
        /// </summary>
        private static List<Point> ComputeConvexHull2D(IList<Point> pts)
        {
            if (pts == null || pts.Count <= 1)
                return pts?.ToList() ?? new List<Point>();

            var sorted = pts.OrderBy(p => p.X).ThenBy(p => p.Y).ToList();

            var lower = new List<Point>();
            foreach (var p in sorted)
            {
                while (lower.Count >= 2 &&
                       Cross(lower[lower.Count - 2], lower[lower.Count - 1], p) <= 0)
                {
                    lower.RemoveAt(lower.Count - 1);
                }
                lower.Add(p);
            }

            var upper = new List<Point>();
            for (int i = sorted.Count - 1; i >= 0; i--)
            {
                var p = sorted[i];
                while (upper.Count >= 2 &&
                       Cross(upper[upper.Count - 2], upper[upper.Count - 1], p) <= 0)
                {
                    upper.RemoveAt(upper.Count - 1);
                }
                upper.Add(p);
            }

            // Remove duplicate endpoints
            if (lower.Count > 0) lower.RemoveAt(lower.Count - 1);
            if (upper.Count > 0) upper.RemoveAt(upper.Count - 1);

            lower.AddRange(upper);
            return lower;
        }

        private static double Cross(Point a, Point b, Point c)
        {
            return (b.X - a.X) * (c.Y - a.Y) - (b.Y - a.Y) * (c.X - a.X);
        }

        /// <summary>
        /// Signed polygon area (positive for CCW). We use absolute value.
        /// </summary>
        private static double PolygonArea(IList<Point> poly)
        {
            int n = poly.Count;
            if (n < 3) return 0;

            double sum = 0;
            for (int i = 0; i < n; i++)
            {
                var p0 = poly[i];
                var p1 = poly[(i + 1) % n];
                sum += p0.X * p1.Y - p1.X * p0.Y;
            }
            return Math.Abs(sum) * 0.5;
        }
    }
}