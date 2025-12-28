using System;
using System.Collections.Generic;
using System.Drawing;
using System.Drawing.Imaging;
using System.Linq;
using System.Windows.Media.Media3D;

public static class BmpHelper
{
    // Assume you already have:
    //  - PlaneBasis / PlaneFitResult
    //  - ProjectPointToPlane2D(Point3D, PlaneBasis)
    //  - ProjectionBitmapResult class (Bitmap + bounds etc.)

    /// <summary>
    /// Project a 3D point cloud onto a plane and rasterize to a bitmap.
    /// White pixels represent “material”, black pixels are voids.
    /// The point radius is chosen automatically from point spacing
    /// if pointRadiusPixels <= 0.
    /// </summary>
    public static ProjectionBitmapResult ProjectToBitmap(
        Point3DCollection points,
        PlaneBasis plane,
        double pixelSize = 1.0,       // 1:1 if your projected units are "pixel-like"
        int paddingPixels = 4,
        int pointRadiusPixels = 0      // <= 0 => auto
    )
    {
        if (points == null || points.Count == 0)
            return null;

        // --- 1. Project to plane (2D) ---
        var projected = new List<PointF>(points.Count);
        foreach (var p in points)
        {
            var q = PointCloudProcessing.ProjectPointToPlane2D(p, plane); // returns System.Windows.Point
            projected.Add(new PointF((float)q.X, (float)q.Y));
        }

        double minX = projected.Min(pt => pt.X);
        double maxX = projected.Max(pt => pt.X);
        double minY = projected.Min(pt => pt.Y);
        double maxY = projected.Max(pt => pt.Y);

        double widthWorld  = maxX - minX;
        double heightWorld = maxY - minY;
        if (widthWorld <= 0 || heightWorld <= 0)
            throw new InvalidOperationException("Degenerate projection bounds.");

        // --- 2. Auto-estimate radius if requested ---
        if (pointRadiusPixels <= 0)
        {
            pointRadiusPixels = AutoEstimatePointRadiusPixels(projected, pixelSize);
        }

        int widthPx  = (int)Math.Ceiling(widthWorld  / pixelSize) + 2 * paddingPixels;
        int heightPx = (int)Math.Ceiling(heightWorld / pixelSize) + 2 * paddingPixels;

        var mask = new bool[widthPx, heightPx];

        // --- 3. Stamp each point as a filled disk of radius r ---
        int r = Math.Max(1, pointRadiusPixels);

        for (int i = 0; i < projected.Count; i++)
        {
            var pt = projected[i];

            int cx = (int)Math.Round((pt.X - minX) / pixelSize) + paddingPixels;
            // Flip Y so bitmap Y grows downward
            int cy = (int)Math.Round((maxY - pt.Y) / pixelSize) + paddingPixels;

            if (cx < 0 || cx >= widthPx || cy < 0 || cy >= heightPx)
                continue;

            for (int dy = -r; dy <= r; dy++)
            {
                int yy = cy + dy;
                if (yy < 0 || yy >= heightPx) continue;

                for (int dx = -r; dx <= r; dx++)
                {
                    int xx = cx + dx;
                    if (xx < 0 || xx >= widthPx) continue;
                    if (dx * dx + dy * dy > r * r) continue; // keep it circular

                    mask[xx, yy] = true;
                }
            }
        }

        // --- 4. Write mask into an 8-bpp bitmap (white = material, black = void) ---
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
                {
                    row[x] = mask[x, y] ? (byte)255 : (byte)0;
                }
            }
        }

        bmp.UnlockBits(data);

        return new ProjectionBitmapResult
        {
            Bitmap    = bmp,
            MinX      = minX,
            MaxX      = maxX,
            MinY      = minY,
            MaxY      = maxY,
            PixelSize = pixelSize,
            Padding   = paddingPixels
        };
    }

    /// <summary>
    /// Estimate a good point radius (in pixels) from the projected point spacing,
    /// so neighboring disks overlap and close the gaps automatically.
    /// </summary>
    private static int AutoEstimatePointRadiusPixels(
        List<PointF> projected,
        double pixelSize)
    {
        if (projected == null || projected.Count < 4)
            return 1;

        double[] xs = projected.Select(p => (double)p.X).OrderBy(v => v).ToArray();
        double[] ys = projected.Select(p => (double)p.Y).OrderBy(v => v).ToArray();

        static double MedianPositiveDiff(double[] arr)
        {
            var diffs = new List<double>();
            for (int i = 1; i < arr.Length; i++)
            {
                double d = arr[i] - arr[i - 1];
                if (d > 0) diffs.Add(d);
            }
            if (diffs.Count == 0) return 1.0;
            diffs.Sort();
            return diffs[diffs.Count / 2];
        }

        double spacingX = MedianPositiveDiff(xs);
        double spacingY = MedianPositiveDiff(ys);
        double spacingWorld = Math.Min(spacingX, spacingY);
        if (spacingWorld <= 0) spacingWorld = 1.0;

        // Convert to pixel units. pixelSize is "world units per pixel".
        double spacingPixels = spacingWorld / Math.Max(pixelSize, 1e-9);

        // Choose radius so neighboring discs overlap nicely.
        double r = 0.6 * spacingPixels; // 0.5–0.7 works well
        int radius = (int)Math.Ceiling(r);

        // Clamp to something reasonable
        radius = Math.Max(1, Math.Min(radius, 10));

        return radius;
    }
}