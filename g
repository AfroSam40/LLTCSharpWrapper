using System;
using System.Collections.Generic;
using System.Drawing;
using System.Drawing.Imaging;
using System.Linq;
using System.Windows;
using System.Windows.Media.Media3D;

namespace LLT
{
    public static class PointCloudProcessing
    {
        // --------------------------------------------------------------------
        // Existing types (shortened here – keep your real definitions)
        // --------------------------------------------------------------------

        public struct PlaneBasis
        {
            public Point3D Origin;   // point on plane
            public Vector3D U;       // in-plane axis 1 (normalized)
            public Vector3D V;       // in-plane axis 2 (normalized)
            public Vector3D Normal;  // plane normal (normalized)
        }

        public class PlaneFitResult
        {
            public double A { get; set; }
            public double B { get; set; }
            public double C { get; set; }

            public Vector3D Normal { get; set; }
            public Point3D Centroid { get; set; }

            public double AverageError { get; set; }
            public List<Point3D> InlierPoints { get; set; } = new List<Point3D>();
        }

        // Your existing 3D→2D projection:
        public static Point ProjectPointToPlane2D(Point3D point, PlaneBasis plane)
        {
            Vector3D vec = point - plane.Origin;
            double x = Vector3D.DotProduct(vec, plane.U);
            double y = Vector3D.DotProduct(vec, plane.V);
            return new Point(x, y);
        }

        // --------------------------------------------------------------------
        // NEW: helper to build PlaneBasis from PlaneFitResult
        // --------------------------------------------------------------------
        public static PlaneBasis BuildPlaneBasisFromFit(PlaneFitResult fit)
        {
            if (fit == null)
                throw new ArgumentNullException(nameof(fit));

            Vector3D n = fit.Normal;
            if (n.LengthSquared < 1e-12)
                n = new Vector3D(0, 0, 1);
            n.Normalize();

            // pick an arbitrary non-parallel vector to build U,V in plane
            Vector3D temp = Math.Abs(n.Z) < 0.9
                ? new Vector3D(0, 0, 1)
                : new Vector3D(0, 1, 0);

            Vector3D u = Vector3D.CrossProduct(temp, n);
            if (u.LengthSquared < 1e-12)
                u = new Vector3D(1, 0, 0);
            u.Normalize();

            Vector3D v = Vector3D.CrossProduct(n, u);
            v.Normalize();

            return new PlaneBasis
            {
                Origin = fit.Centroid,
                U = u,
                V = v,
                Normal = n
            };
        }

        // --------------------------------------------------------------------
        // ProjectionBitmapResult (unchanged, except made public here)
        // --------------------------------------------------------------------
        public class ProjectionBitmapResult
        {
            public Bitmap Bitmap { get; set; }

            public double MinX { get; set; }
            public double MaxX { get; set; }
            public double MinY { get; set; }
            public double MaxY { get; set; }

            public double PixelSize { get; set; }

            internal int Padding { get; set; }

            public void WorldToPixel(double x, double y, out int ix, out int iy)
            {
                double widthWorld = MaxX - MinX;
                double heightWorld = MaxY - MinY;

                int widthPx = (int)Math.Ceiling(widthWorld / PixelSize) + 2 * Padding;
                int heightPx = (int)Math.Ceiling(heightWorld / PixelSize) + 2 * Padding;

                ix = (int)Math.Round((x - MinX) / PixelSize) + Padding;
                iy = (int)Math.Round((MaxY - y) / PixelSize) + Padding;

                ix = Math.Max(0, Math.Min(widthPx - 1, ix));
                iy = Math.Max(0, Math.Min(heightPx - 1, iy));
            }
        }

        // --------------------------------------------------------------------
        // NEW OVERLOAD: ProjectToBitmap using PlaneFitResult
        // --------------------------------------------------------------------
        public static ProjectionBitmapResult ProjectToBitmap(
            Point3DCollection points,
            PlaneFitResult fit,
            double pixelSize = 0.05,
            int paddingPixels = 5,
            int pointRadiusPixels = 1)
        {
            if (fit == null)
                throw new ArgumentNullException(nameof(fit));

            var basis = BuildPlaneBasisFromFit(fit);
            return ProjectToBitmap(points, basis, pixelSize, paddingPixels, pointRadiusPixels);
        }

        // --------------------------------------------------------------------
        // Existing implementation using PlaneBasis (can stay public or private)
        // --------------------------------------------------------------------
        public static ProjectionBitmapResult ProjectToBitmap(
            Point3DCollection points,
            PlaneBasis plane,
            double pixelSize = 0.05,
            int paddingPixels = 5,
            int pointRadiusPixels = 1)
        {
            if (points == null || points.Count == 0)
                return null;

            // 1) project to plane
            var projected = new List<Point>(points.Count);
            foreach (var p in points)
                projected.Add(ProjectPointToPlane2D(p, plane));

            double minX = projected.Min(pt => pt.X);
            double maxX = projected.Max(pt => pt.X);
            double minY = projected.Min(pt => pt.Y);
            double maxY = projected.Max(pt => pt.Y);

            double widthWorld = maxX - minX;
            double heightWorld = maxY - minY;

            if (widthWorld <= 0 || heightWorld <= 0)
                throw new InvalidOperationException("Degenerate projection bounds.");

            int widthPx = (int)Math.Ceiling(widthWorld / pixelSize) + 2 * paddingPixels;
            int heightPx = (int)Math.Ceiling(heightWorld / pixelSize) + 2 * paddingPixels;

            var mask = new bool[widthPx, heightPx];

            for (int i = 0; i < projected.Count; i++)
            {
                var pt = projected[i];

                int cx = (int)Math.Round((pt.X - minX) / pixelSize) + paddingPixels;
                int cy = (int)Math.Round((maxY - pt.Y) / pixelSize) + paddingPixels;

                if (cx < 0 || cx >= widthPx || cy < 0 || cy >= heightPx)
                    continue;

                int r = Math.Max(0, pointRadiusPixels);
                for (int dy = -r; dy <= r; dy++)
                {
                    int yy = cy + dy;
                    if (yy < 0 || yy >= heightPx) continue;

                    for (int dx = -r; dx <= r; dx++)
                    {
                        int xx = cx + dx;
                        if (xx < 0 || xx >= widthPx) continue;
                        if (dx * dx + dy * dy > r * r) continue; // circular brush

                        mask[xx, yy] = true;
                    }
                }
            }

            var bmp = new Bitmap(widthPx, heightPx, PixelFormat.Format8bppIndexed);
            var pal = bmp.Palette;
            for (int i = 0; i < 256; i++)
                pal.Entries[i] = Color.FromArgb(i, i, i);
            bmp.Palette = pal;

            var rect = new Rectangle(0, 0, widthPx, heightPx);
            var data = bmp.LockBits(rect, ImageLockMode.WriteOnly, PixelFormat.Format8bppIndexed);

            unsafe
            {
                byte* basePtr = (byte*)data.Scan0;
                int stride = data.Stride;

                for (int y = 0; y < heightPx; y++)
                {
                    byte* row = basePtr + y * stride;
                    for (int x = 0; x < widthPx; x++)
                        row[x] = mask[x, y] ? (byte)255 : (byte)0;
                }
            }

            bmp.UnlockBits(data);

            return new ProjectionBitmapResult
            {
                Bitmap = bmp,
                MinX = minX,
                MaxX = maxX,
                MinY = minY,
                MaxY = maxY,
                PixelSize = pixelSize,
                Padding = paddingPixels
            };
        }
    }
}