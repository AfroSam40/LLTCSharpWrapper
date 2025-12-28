using OpenCvSharp;
using System;
using System.Linq;
using System.Collections.Generic;

public static class RectWallsFromContour
{
    // Returns 4 line segments (each as (p1,p2)): left, right, top, bottom (order noted below)
    public static (Point2f A, Point2f B)[] FindRectangleWalls(Point[] contour, double clipPercent = 2.0)
    {
        if (contour == null || contour.Length < 10)
            throw new ArgumentException("Contour too small.");

        // --- 1) PCA to estimate dominant rectangle axis ---
        // Build Nx2 float matrix of points
        using var ptsMat = new Mat(contour.Length, 2, MatType.CV_32F);
        for (int i = 0; i < contour.Length; i++)
        {
            ptsMat.Set(i, 0, (float)contour[i].X);
            ptsMat.Set(i, 1, (float)contour[i].Y);
        }

        using var mean = new Mat();
        using var evecs = new Mat();
        Cv2.PCACompute(ptsMat, mean, evecs);

        // First eigenvector is dominant direction
        float vx = evecs.At<float>(0, 0);
        float vy = evecs.At<float>(0, 1);
        double angle = Math.Atan2(vy, vx);

        // Rotation to align dominant axis with +X (rotate by -angle)
        double ca = Math.Cos(-angle);
        double sa = Math.Sin(-angle);

        // Mean center
        float cx = mean.At<float>(0, 0);
        float cy = mean.At<float>(0, 1);

        // Rotate all points into PCA-aligned space
        float[] xs = new float[contour.Length];
        float[] ys = new float[contour.Length];
        for (int i = 0; i < contour.Length; i++)
        {
            float x = contour[i].X - cx;
            float y = contour[i].Y - cy;
            float xr = (float)(x * ca - y * sa);
            float yr = (float)(x * sa + y * ca);
            xs[i] = xr;
            ys[i] = yr;
        }

        // --- 2) Robust bounds (ignore ear outliers) ---
        // Use e.g. 2nd/98th percentile instead of min/max
        float left   = Percentile(xs, clipPercent);
        float right  = Percentile(xs, 100.0 - clipPercent);
        float top    = Percentile(ys, clipPercent);
        float bottom = Percentile(ys, 100.0 - clipPercent);

        // Also choose line extents (span) robustly
        float xMin = left, xMax = right;
        float yMin = top,  yMax = bottom;

        // --- 3) Create 4 axis-aligned segments in rotated space ---
        // left wall:   x = left,  y from yMin..yMax
        // right wall:  x = right, y from yMin..yMax
        // top wall:    y = top,   x from xMin..xMax
        // bottom wall: y = bottom,x from xMin..xMax
        var leftSegR   = (new Point2f(left,  yMin), new Point2f(left,  yMax));
        var rightSegR  = (new Point2f(right, yMin), new Point2f(right, yMax));
        var topSegR    = (new Point2f(xMin,  top),  new Point2f(xMax,  top));
        var bottomSegR = (new Point2f(xMin,  bottom),new Point2f(xMax, bottom));

        // --- 4) Rotate segments back to original coordinates ---
        // inverse rotation is +angle
        double cb = Math.Cos(angle);
        double sb = Math.Sin(angle);

        Point2f Unrotate(Point2f p)
        {
            float x = p.X;
            float y = p.Y;
            float xo = (float)(x * cb - y * sb) + cx;
            float yo = (float)(x * sb + y * cb) + cy;
            return new Point2f(xo, yo);
        }

        (Point2f, Point2f) Back((Point2f A, Point2f B) s) => (Unrotate(s.A), Unrotate(s.B));

        // Return in a consistent order:
        // 0=left, 1=right, 2=top, 3=bottom
        return new[]
        {
            Back(leftSegR),
            Back(rightSegR),
            Back(topSegR),
            Back(bottomSegR),
        };
    }

    private static float Percentile(float[] data, double p)
    {
        var sorted = data.OrderBy(v => v).ToArray();
        if (sorted.Length == 0) return 0;

        double pos = (p / 100.0) * (sorted.Length - 1);
        int i = (int)Math.Floor(pos);
        int j = (int)Math.Ceiling(pos);
        if (i == j) return sorted[i];

        float a = sorted[i];
        float b = sorted[j];
        float t = (float)(pos - i);
        return a + (b - a) * t;
    }
}



var walls = RectWallsFromContour.FindRectangleWalls(contour, clipPercent: 2.0);
// walls[0]=left, walls[1]=right, walls[2]=top, walls[3]=bottom
Cv2.Line(img, (Point)walls[0].A, (Point)walls[0].B, Scalar.Lime, 2);
Cv2.Line(img, (Point)walls[1].A, (Point)walls[1].B, Scalar.Lime, 2);
Cv2.Line(img, (Point)walls[2].A, (Point)walls[2].B, Scalar.Lime, 2);
Cv2.Line(img, (Point)walls[3].A, (Point)walls[3].B, Scalar.Lime, 2);