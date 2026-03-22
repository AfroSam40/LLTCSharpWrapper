using System;
using System.Collections.Generic;
using System.Drawing;
using System.Drawing.Imaging;
using System.Linq;
using System.Windows.Media.Media3D;

public static ProjectionBitmapResult ProjectToBitmap(
    PointCloud cloud,
    PlaneFitResult fit,
    double pixelSize = 0.05,
    int paddingPixels = 0,
    int pointRadiusPixels = 0)
{
    var points = cloud?.Points;
    if (points == null || points.Count == 0) throw new ArgumentException("Empty point cloud.", nameof(cloud));
    if (fit == null) throw new ArgumentNullException(nameof(fit));
    if (pixelSize <= 0) throw new ArgumentOutOfRangeException(nameof(pixelSize));

    var n = fit.Normal;
    if (n.LengthSquared < 1e-18) n = new Vector3D(0, 0, 1);
    n.Normalize();

    var axis = Math.Abs(n.Z) < 0.9 ? new Vector3D(0, 0, 1) : new Vector3D(0, 1, 0);
    var uAxis = Vector3D.CrossProduct(axis, n);
    if (uAxis.LengthSquared < 1e-18) uAxis = Vector3D.CrossProduct(new Vector3D(1, 0, 0), n);
    uAxis.Normalize();

    var vAxis = Vector3D.CrossProduct(n, uAxis);
    vAxis.Normalize();

    var o = fit.Centroid;

    var uv = points.Select(p =>
    {
        var d = p - o;
        return (U: Vector3D.DotProduct(d, uAxis), V: Vector3D.DotProduct(d, vAxis));
    }).ToArray();

    double minU = uv.Min(t => t.U), maxU = uv.Max(t => t.U);
    double minV = uv.Min(t => t.V), maxV = uv.Max(t => t.V);
    double wWorld = maxU - minU, hWorld = maxV - minV;
    if (wWorld <= 0 || hWorld <= 0) throw new InvalidOperationException("Degenerate projection bounds.");

    int w = (int)Math.Ceiling(wWorld / pixelSize) + 2 * paddingPixels;
    int h = (int)Math.Ceiling(hWorld / pixelSize) + 2 * paddingPixels;
    if (w < 1 || h < 1) throw new InvalidOperationException("Invalid bitmap size.");

    if (pointRadiusPixels <= 0)
    {
        static double MedianPositiveDiff(double[] a)
        {
            Array.Sort(a);
            var diffs = new List<double>(Math.Max(0, a.Length - 1));
            for (int i = 1; i < a.Length; i++)
            {
                double d = a[i] - a[i - 1];
                if (d > 0) diffs.Add(d);
            }
            if (diffs.Count == 0) return 1.0;
            diffs.Sort();
            return diffs[diffs.Count / 2];
        }

        var us = uv.Select(t => t.U).ToArray();
        var vs = uv.Select(t => t.V).ToArray();
        double sx = MedianPositiveDiff(us);
        double sy = MedianPositiveDiff(vs);
        double spacingPx = Math.Min(sx, sy) / Math.Max(pixelSize, 1e-12);
        pointRadiusPixels = Math.Max(1, Math.Min(12, (int)Math.Ceiling(0.6 * spacingPx)));
    }

    var mask = new bool[w, h];
    int r = Math.Max(1, pointRadiusPixels);
    int r2 = r * r;

    for (int i = 0; i < uv.Length; i++)
    {
        int cx = (int)Math.Round((uv[i].U - minU) / pixelSize) + paddingPixels;
        int cy = (int)Math.Round((maxV - uv[i].V) / pixelSize) + paddingPixels;

        if ((uint)cx >= (uint)w || (uint)cy >= (uint)h) continue;

        for (int dy = -r; dy <= r; dy++)
        {
            int yy = cy + dy;
            if ((uint)yy >= (uint)h) continue;
            int dy2 = dy * dy;

            for (int dx = -r; dx <= r; dx++)
            {
                if (dx * dx + dy2 > r2) continue;
                int xx = cx + dx;
                if ((uint)xx >= (uint)w) continue;
                mask[xx, yy] = true;
            }
        }
    }

    var bmp = new Bitmap(w, h, PixelFormat.Format8bppIndexed);
    var pal = bmp.Palette;
    for (int i = 0; i < 256; i++) pal.Entries[i] = Color.FromArgb(i, i, i);
    bmp.Palette = pal;

    var rect = new Rectangle(0, 0, w, h);
    var data = bmp.LockBits(rect, ImageLockMode.WriteOnly, PixelFormat.Format8bppIndexed);

    unsafe
    {
        byte* basePtr = (byte*)data.Scan0;
        int stride = data.Stride;

        for (int y = 0; y < h; y++)
        {
            byte* row = basePtr + y * stride;
            for (int x = 0; x < w; x++)
                row[x] = mask[x, y] ? (byte)255 : (byte)0;
        }
    }

    bmp.UnlockBits(data);

    return new ProjectionBitmapResult
    {
        Bitmap = bmp,
        MinX = minU,
        MaxX = maxU,
        MinY = minV,
        MaxY = maxV,
        PixelSize = pixelSize,
        Padding = paddingPixels,
        Origin = o,
        U = uAxis,
        V = vAxis,
        Normal = n
    };
}