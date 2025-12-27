using System;
using System.Collections.Generic;
using System.Linq;
using System.Windows;
using System.Windows.Media;
using System.Windows.Media.Media3D;
using HelixToolkit.Wpf;

namespace PointCloudUtils
{
    public static class PlaneVisualBuilder
    {
        /// <summary>
        /// Build a plane patch visual from a PlaneFitResult using the convex hull
        /// of its inlier points. No MeshBuilder; uses MeshGeometry3D directly.
        /// </summary>
        /// <param name="plane">The fitted plane result.</param>
        /// <param name="paddingFactor">
        /// > 1.0 to slightly expand the hull around its centroid (e.g. 1.1).
        /// = 1.0 for no padding.
        /// </param>
        /// <param name="fill">Brush for the plane.</param>
        /// <returns>ModelVisual3D you can add to the viewport, or null if not enough points.</returns>
        public static ModelVisual3D BuildPlanePatchVisual(
            PlaneFitResult plane,
            double paddingFactor,
            Brush fill)
        {
            var pts = plane.InlierPoints;
            if (pts == null || pts.Count < 3)
                return null;

            // 1) Build orthonormal basis (u, v, n)
            Vector3D n = plane.Normal;
            if (n.LengthSquared < 1e-12)
                n = new Vector3D(0, 0, 1);
            n.Normalize();

            // Pick an arbitrary "up" not collinear with n
            Vector3D up = Math.Abs(n.Z) < 0.9
                ? new Vector3D(0, 0, 1)
                : new Vector3D(1, 0, 0);

            Vector3D u = Vector3D.CrossProduct(n, up);
            if (u.LengthSquared < 1e-12)
                u = new Vector3D(1, 0, 0);
            u.Normalize();

            Vector3D v = Vector3D.CrossProduct(n, u);
            v.Normalize();

            Point3D c = plane.Centroid;

            // 2) Project inliers into (u, v) coordinates centered at centroid
            var proj2D = new List<Point>(pts.Count);
            foreach (var p in pts)
            {
                Vector3D d = p - c;
                double ux = Vector3D.DotProduct(d, u);
                double vy = Vector3D.DotProduct(d, v);
                proj2D.Add(new Point(ux, vy));
            }

            // 3) Compute convex hull in 2D (monotone chain)
            var hull = ComputeConvexHull(proj2D);
            if (hull.Count < 3)
                return null;

            // 4) Optional padding: expand hull around its centroid
            if (paddingFactor > 1.0)
            {
                double cx = hull.Average(p => p.X);
                double cy = hull.Average(p => p.Y);

                for (int i = 0; i < hull.Count; i++)
                {
                    var p = hull[i];
                    double dx = p.X - cx;
                    double dy = p.Y - cy;
                    hull[i] = new Point(
                        cx + dx * paddingFactor,
                        cy + dy * paddingFactor);
                }
            }

            // 5) Build MeshGeometry3D from hull (triangle fan)
            var mesh = new MeshGeometry3D
            {
                Positions = new Point3DCollection(),
                TriangleIndices = new Int32Collection()
            };

            // Convert hull 2D -> 3D world positions on the plane
            foreach (var h in hull)
            {
                Point3D wp = c + h.X * u + h.Y * v;
                mesh.Positions.Add(wp);
            }

            int nVerts = mesh.Positions.Count;
            if (nVerts < 3)
                return null;

            // Triangle fan: (0, i, i+1)
            for (int i = 1; i < nVerts - 1; i++)
            {
                mesh.TriangleIndices.Add(0);
                mesh.TriangleIndices.Add(i);
                mesh.TriangleIndices.Add(i + 1);
            }

            // 6) Wrap into GeometryModel3D and ModelVisual3D
            var mat = new DiffuseMaterial(fill);
            var gm = new GeometryModel3D(mesh, mat)
            {
                BackMaterial = mat
            };

            var visual = new ModelVisual3D { Content = gm };
            return visual;
        }

        /// <summary>
        /// 2D convex hull via monotone chain; returns hull in CCW order.
        /// Uses System.Windows.Point.
        /// </summary>
        private static List<Point> ComputeConvexHull(List<Point> pts)
        {
            var points = pts
                .OrderBy(p => p.X)
                .ThenBy(p => p.Y)
                .ToList();

            if (points.Count <= 1)
                return new List<Point>(points);

            List<Point> lower = new List<Point>();
            foreach (var p in points)
            {
                while (lower.Count >= 2 &&
                       Cross(lower[lower.Count - 2], lower[lower.Count - 1], p) <= 0)
                {
                    lower.RemoveAt(lower.Count - 1);
                }
                lower.Add(p);
            }

            List<Point> upper = new List<Point>();
            for (int i = points.Count - 1; i >= 0; i--)
            {
                var p = points[i];
                while (upper.Count >= 2 &&
                       Cross(upper[upper.Count - 2], upper[upper.Count - 1], p) <= 0)
                {
                    upper.RemoveAt(upper.Count - 1);
                }
                upper.Add(p);
            }

            // Remove last point of each list because it’s the start of the other list
            upper.RemoveAt(upper.Count - 1);
            lower.RemoveAt(lower.Count - 1);

            lower.AddRange(upper);
            return lower;
        }

        private static double Cross(Point o, Point a, Point b)
        {
            return (a.X - o.X) * (b.Y - o.Y) - (a.Y - o.Y) * (b.X - o.X);
        }
    }
}