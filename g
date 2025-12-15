using System;
using System.Collections.Generic;
using System.Linq;
using System.Windows;
using System.Windows.Media;
using System.Windows.Media.Media3D;
using HelixToolkit.Wpf;

namespace LLT
{
    public class PlaneFitResult
    {
        public Point3D Origin { get; set; }
        public Vector3D Normal { get; set; }   // Should be normalized
    }

    /// <summary>
    /// Represents one slice (band) of the blob along the fitted plane normal.
    /// Uses a 2D convex hull projected onto the slice plane.
    /// </summary>
    public class BlobSlice
    {
        /// <summary>Lower height (along plane normal) of this slice, in mm.</summary>
        public double Z0 { get; set; }

        /// <summary>Upper height (along plane normal) of this slice, in mm.</summary>
        public double Z1 { get; set; }

        /// <summary>Area of this slice’s cross section (mm²), from convex hull.</summary>
        public double Area { get; set; }

        /// <summary>Volume contribution from this slice (mm³) = Area * thickness.</summary>
        public double VolumeContribution { get; set; }

        /// <summary>Center of the slice polygon in world coordinates.</summary>
        public Point3D CenterWorld { get; set; }

        /// <summary>Normal of the base plane used for slicing (normalized).</summary>
        public Vector3D Normal { get; set; }

        /// <summary>Hull vertices in world coordinates, lying in the slice plane.</summary>
        public Point3D[] HullWorld { get; set; } = Array.Empty<Point3D>();
    }

    public static class BlobSlicing3D
    {
        /// <summary>
        /// Estimate volume of a blob by slicing a point cloud along a fitted base plane
        /// and using 2D convex hulls for each slice cross-section.
        /// Returns total volume and a list of slices with hull polygons in 3D.
        /// </summary>
        /// <param name="points">Point cloud of the blob and base region (mm).</param>
        /// <param name="basePlane">Plane on which the blob is sitting.</param>
        /// <param name="sliceThickness">Distance between slices along plane normal (mm).</param>
        /// <param name="maxHeight">
        /// Maximum height above base plane to consider (mm).
        /// If <= 0, it is auto-chosen from the cloud.
        /// </param>
        public static (double totalVolume, List<BlobSlice> slices)
            EstimateBlobVolumeByHullSlices(
                Point3DCollection points,
                PlaneFitResult basePlane,
                double sliceThickness,
                double maxHeight = 0.0)
        {
            if (points == null) throw new ArgumentNullException(nameof(points));
            if (points.Count < 3) return (0.0, new List<BlobSlice>());
            if (sliceThickness <= 0) throw new ArgumentOutOfRangeException(nameof(sliceThickness));

            // Normalize normal and make sure heights "above" plane are positive.
            Vector3D n = basePlane.Normal;
            if (n.LengthSquared < 1e-12)
                throw new ArgumentException("Plane normal is zero.", nameof(basePlane));

            n.Normalize();

            // If normal points "down" in Z, flip it so that "up" is positive.
            if (n.Z < 0)
                n = -n;

            Point3D origin = basePlane.Origin;

            // Build orthonormal basis (ex, ey, n) for the plane.
            Vector3D tmp = Math.Abs(n.Z) < 0.9 ? new Vector3D(0, 0, 1) : new Vector3D(0, 1, 0);
            Vector3D ex = Vector3D.CrossProduct(tmp, n);
            ex.Normalize();
            Vector3D ey = Vector3D.CrossProduct(n, ex);
            ey.Normalize();

            // Precompute local coordinates (u, v, h) for each point.
            int count = points.Count;
            var locals = new (double u, double v, double h)[count];
            double maxH = double.NegativeInfinity;

            for (int i = 0; i < count; i++)
            {
                Vector3D v3 = points[i] - origin;
                double h = Vector3D.DotProduct(v3, n);      // height along normal
                double u = Vector3D.DotProduct(v3, ex);     // local X in plane
                double v = Vector3D.DotProduct(v3, ey);     // local Y in plane

                locals[i] = (u, v, h);

                if (h > maxH) maxH = h;
            }

            if (maxHeight <= 0 || maxHeight > maxH)
                maxHeight = maxH;

            int sliceCount = (int)Math.Ceiling(maxHeight / sliceThickness);

            double totalVolume = 0.0;
            var slices = new List<BlobSlice>();

            for (int s = 0; s < sliceCount; s++)
            {
                double z0 = s * sliceThickness;
                double z1 = z0 + sliceThickness;

                // Collect 2D points for this slice [z0, z1)
                var slice2D = new List<Point>();

                for (int i = 0; i < count; i++)
                {
                    double h = locals[i].h;
                    if (h >= z0 && h < z1)
                    {
                        slice2D.Add(new Point(locals[i].u, locals[i].v));
                    }
                }

                if (slice2D.Count < 3)
                {
                    // Not enough points to form a polygon.
                    continue;
                }

                // Compute convex hull in 2D local plane coordinates.
                var hull2D = ComputeConvexHull(slice2D);
                if (hull2D.Count < 3)
                    continue;

                // Area and centroid of hull (2D).
                double area2D = ComputePolygonArea(hull2D);
                if (area2D <= 0)
                    continue;

                Point centroid2D = ComputePolygonCentroid(hull2D, area2D);

                // Volume contribution for this slice.
                double volumeSlice = area2D * sliceThickness;
                totalVolume += volumeSlice;

                // Map hull to world coordinates at mid-height of slice.
                double hMid = 0.5 * (z0 + z1);
                var hullWorld = new Point3D[hull2D.Count];
                for (int i = 0; i < hull2D.Count; i++)
                {
                    var p2 = hull2D[i];
                    Vector3D offset = ex * p2.X + ey * p2.Y + n * hMid;
                    hullWorld[i] = origin + offset;
                }

                // Center in world coordinates.
                Vector3D centerOffset = ex * centroid2D.X + ey * centroid2D.Y + n * hMid;
                Point3D centerWorld = origin + centerOffset;

                slices.Add(new BlobSlice
                {
                    Z0 = z0,
                    Z1 = z1,
                    Area = area2D,
                    VolumeContribution = volumeSlice,
                    CenterWorld = centerWorld,
                    Normal = n,
                    HullWorld = hullWorld
                });
            }

            return (totalVolume, slices);
        }

        /// <summary>
        /// Build slice visuals (one polygon per slice) from BlobSlice hulls.
        /// Each slice is rendered as a filled polygon using MeshGeometry3D
        /// with a triangle fan (center + hull vertices).
        /// </summary>
        /// <param name="parent">ModelVisual3D to host the slice models.</param>
        /// <param name="slices">Slices from EstimateBlobVolumeByHullSlices.</param>
        /// <param name="color">Base color of the slice polygons.</param>
        /// <param name="opacity">Polygon opacity (0..1).</param>
        public static void BuildHullSliceVisuals(
            ModelVisual3D parent,
            IEnumerable<BlobSlice> slices,
            Color color,
            double opacity = 0.3)
        {
            if (parent == null) throw new ArgumentNullException(nameof(parent));
            if (slices == null) throw new ArgumentNullException(nameof(slices));

            parent.Children.Clear();

            byte a = (byte)(Math.Max(0, Math.Min(1, opacity)) * 255);
            var brush = new SolidColorBrush(Color.FromArgb(a, color.R, color.G, color.B));
            brush.Freeze();

            var material = new DiffuseMaterial(brush) { };
            var backMaterial = material;

            foreach (var slice in slices)
            {
                var hull = slice.HullWorld;
                if (hull == null || hull.Length < 3)
                    continue;

                var positions = new Point3DCollection();
                var indices = new Int32Collection();

                // Center vertex at index 0
                positions.Add(slice.CenterWorld);

                // Hull vertices at indices [1..N]
                for (int i = 0; i < hull.Length; i++)
                    positions.Add(hull[i]);

                int n = hull.Length;
                for (int i = 0; i < n; i++)
                {
                    int i1 = i + 1;
                    int i2 = (i + 1) % n + 1;
                    indices.Add(0);
                    indices.Add(i1);
                    indices.Add(i2);
                }

                var mesh = new MeshGeometry3D
                {
                    Positions = positions,
                    TriangleIndices = indices
                };
                mesh.Freeze();

                var geom = new GeometryModel3D
                {
                    Geometry = mesh,
                    Material = material,
                    BackMaterial = backMaterial
                };

                var mv = new ModelVisual3D { Content = geom };
                parent.Children.Add(mv);
            }
        }

        #region Geometry helpers

        /// <summary>
        /// Standard monotone chain convex hull in 2D.
        /// Input: list of points (may contain duplicates).
        /// Output: hull in CCW order, no duplicate last point.
        /// </summary>
        private static List<Point> ComputeConvexHull(IList<Point> points)
        {
            // Deduplicate
            var pts = points.Distinct().ToList();
            pts.Sort((a, b) =>
            {
                int cmp = a.X.CompareTo(b.X);
                return (cmp != 0) ? cmp : a.Y.CompareTo(b.Y);
            });

            if (pts.Count <= 1)
                return new List<Point>(pts);

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

            // Concatenate lower and upper to form full hull; omit last because it repeats start
            lower.RemoveAt(lower.Count - 1);
            upper.RemoveAt(upper.Count - 1);
            lower.AddRange(upper);
            return lower;
        }

        private static double Cross(Point a, Point b, Point c)
        {
            // Cross product of AB x AC
            return (b.X - a.X) * (c.Y - a.Y) - (b.Y - a.Y) * (c.X - a.X);
        }

        /// <summary>
        /// Shoelace formula for polygon area (absolute value).
        /// Points must be in order (e.g. convex hull).
        /// </summary>
        private static double ComputePolygonArea(IList<Point> poly)
        {
            int n = poly.Count;
            if (n < 3) return 0;

            double sum = 0.0;
            for (int i = 0; i < n; i++)
            {
                var p0 = poly[i];
                var p1 = poly[(i + 1) % n];
                sum += p0.X * p1.Y - p1.X * p0.Y;
            }
            return Math.Abs(sum) * 0.5;
        }

        /// <summary>
        /// Centroid of a 2D polygon given its vertices and signed area.
        /// Area must be positive (use ComputePolygonArea or abs).
        /// </summary>
        private static Point ComputePolygonCentroid(IList<Point> poly, double area)
        {
            int n = poly.Count;
            if (n < 3 || area == 0) return new Point(0, 0);

            double cx = 0.0, cy = 0.0;
            for (int i = 0; i < n; i++)
            {
                var p0 = poly[i];
                var p1 = poly[(i + 1) % n];
                double cross = p0.X * p1.Y - p1.X * p0.Y;
                cx += (p0.X + p1.X) * cross;
                cy += (p0.Y + p1.Y) * cross;
            }

            double factor = 1.0 / (6.0 * area);
            return new Point(cx * factor, cy * factor);
        }

        #endregion
    }
}