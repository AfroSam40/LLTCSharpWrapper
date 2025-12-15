using System;
using System.Collections.Generic;
using System.Linq;
using System.Windows;
using System.Windows.Media.Media3D;

namespace LLT
{
    public static partial class PointCloudProcessing
    {
        /// <summary>
        /// Estimate blob volume by slicing along a fitted base plane and
        /// using a 2D convex hull area per slice (instead of best-fit circle).
        /// </summary>
        /// <param name="points">Blob point cloud.</param>
        /// <param name="basePlane">Plane the blob is sitting on.</param>
        /// <param name="sliceThickness">Slice height along plane normal.</param>
        /// <param name="minPointsPerSlice">Min points needed to accept a slice.</param>
        /// <param name="slices">Output slice descriptors.</param>
        /// <returns>Estimated volume in the same cubic units as the input.</returns>
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

            // ---------- 1. Orthonormal basis (u, v, n) ----------
            Vector3D n = basePlane.Normal;
            if (n.LengthSquared < 1e-12)
                throw new ArgumentException("Base plane normal is zero.", nameof(basePlane));

            n.Normalize();

            // Choose some axis not parallel to n, build u, then v = n x u
            Vector3D temp = Math.Abs(n.Z) < 0.9
                ? new Vector3D(0, 0, 1)
                : new Vector3D(1, 0, 0);

            Vector3D u = Vector3D.CrossProduct(temp, n);
            if (u.LengthSquared < 1e-12)
                u = new Vector3D(1, 0, 0);
            u.Normalize();

            Vector3D v = Vector3D.CrossProduct(n, u);
            v.Normalize();

            // Origin for local coords
            Point3D origin = basePlane.Centroid;

            // ---------- 2. Transform all points to local (u, v, h) ----------
            var local = new List<(double U, double V, double H)>(points.Count);

            foreach (var p in points)
            {
                Vector3D d = p - origin;
                double h = Vector3D.DotProduct(d, n);   // height above plane

                // ignore points clearly below plane (numerical noise)
                if (h < -1e-6)
                    continue;

                double uu = Vector3D.DotProduct(d, u);
                double vv = Vector3D.DotProduct(d, v);

                local.Add((uu, vv, h));
            }

            if (local.Count == 0)
                return 0.0;

            double minH = local.Min(t => t.H);
            double maxH = local.Max(t => t.H);
            if (minH < 0) minH = 0;    // treat slightly negative as on the plane

            int sliceCount = (int)Math.Ceiling((maxH - minH) / sliceThickness);
            if (sliceCount <= 0)
                return 0.0;

            // Buckets of 2D points per slice (in UV plane)
            var sliceBuckets = new List<List<Point>>(sliceCount);
            for (int i = 0; i < sliceCount; i++)
                sliceBuckets.Add(new List<Point>());

            foreach (var (U, V, H) in local)
            {
                int idx = (int)((H - minH) / sliceThickness);
                if (idx < 0) idx = 0;
                if (idx >= sliceCount) idx = sliceCount - 1;

                sliceBuckets[idx].Add(new Point(U, V));
            }

            // ---------- 3. For each slice: convex hull area -> volume ----------
            double totalVolume = 0.0;

            for (int i = 0; i < sliceCount; i++)
            {
                var pts2D = sliceBuckets[i];
                if (pts2D.Count < minPointsPerSlice)
                    continue;

                double area = ComputeConvexHullArea(pts2D);
                if (area <= 0)
                    continue;

                double h0 = minH + i * sliceThickness;
                double h1 = h0 + sliceThickness;
                double hCenter = 0.5 * (h0 + h1);

                // slice center in world coords
                Point3D centerWorld = origin + n * hCenter;

                // Equivalent radius if you still want one (for BlobSlice visual)
                double radius = Math.Sqrt(area / Math.PI);

                var slice = new BlobSlice
                {
                    H0 = h0,
                    H1 = h1,
                    HCenter = hCenter,
                    CenterWorld = centerWorld,
                    Normal = n,
                    Radius = radius,
                    Area = area,
                    PointCount = pts2D.Count
                };

                slices.Add(slice);

                totalVolume += area * sliceThickness;
            }

            return totalVolume;
        }

        // ========== 2D convex hull helpers (monotone chain) ==========

        private static double ComputeConvexHullArea(List<Point> pts)
        {
            if (pts == null || pts.Count < 3)
                return 0.0;

            var hull = BuildConvexHull(pts);
            if (hull.Count < 3)
                return 0.0;

            return Math.Abs(ShoelaceArea(hull));
        }

        private static List<Point> BuildConvexHull(List<Point> points)
        {
            // Andrew's monotone chain
            var pts = points
                .OrderBy(p => p.X)
                .ThenBy(p => p.Y)
                .ToList();

            var lower = new List<Point>();
            foreach (var p in pts)
            {
                while (lower.Count >= 2 && Cross(lower[lower.Count - 2], lower[lower.Count - 1], p) <= 0)
                    lower.RemoveAt(lower.Count - 1);
                lower.Add(p);
            }

            var upper = new List<Point>();
            for (int i = pts.Count - 1; i >= 0; i--)
            {
                var p = pts[i];
                while (upper.Count >= 2 && Cross(upper[upper.Count - 2], upper[upper.Count - 1], p) <= 0)
                    upper.RemoveAt(upper.Count - 1);
                upper.Add(p);
            }

            lower.RemoveAt(lower.Count - 1);
            upper.RemoveAt(upper.Count - 1);
            lower.AddRange(upper);
            return lower;
        }

        private static double Cross(Point o, Point a, Point b)
        {
            return (a.X - o.X) * (b.Y - o.Y) - (a.Y - o.Y) * (b.X - o.X);
        }

        private static double ShoelaceArea(List<Point> poly)
        {
            double sum = 0.0;
            int n = poly.Count;
            for (int i = 0; i < n; i++)
            {
                var p0 = poly[i];
                var p1 = poly[(i + 1) % n];
                sum += p0.X * p1.Y - p1.X * p0.Y;
            }
            return 0.5 * sum;
        }
    }
}