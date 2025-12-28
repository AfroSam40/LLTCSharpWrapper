using System;
using System.Collections.Generic;
using System.Linq;
using OpenCvSharp;

public static class RectSideFitter
{
    /// <summary>
    /// From a single contour that roughly outlines a rectangle-with-ear shape,
    /// group contour points into top/bottom/left/right bands by a thickness
    /// in pixels, fit a line to each band, and draw those lines on the image.
    /// Returns the 4 fitted lines (vx, vy, x0, y0) – some may be (0,0,0,0)
    /// if not enough points were found for that side.
    /// </summary>
    public static (Vec4f top, Vec4f right, Vec4f bottom, Vec4f left)
        FitAndDrawRectSidesFromContour(
            Mat img,
            Point[] contour,
            float bandThickness = 5f,
            Scalar? lineColor = null,
            int thickness = 2)
    {
        if (img == null) throw new ArgumentNullException(nameof(img));
        if (contour == null || contour.Length < 10)
            throw new ArgumentException("Contour is null or too small.", nameof(contour));

        Scalar color = lineColor ?? new Scalar(0, 0, 255); // red by default

        // --- 1. Basic bounds of the contour ---
        int minX = contour.Min(p => p.X);
        int maxX = contour.Max(p => p.X);
        int minY = contour.Min(p => p.Y);
        int maxY = contour.Max(p => p.Y);

        // --- 2. Split contour points into bands ---
        var topPts    = new List<Point2f>();
        var bottomPts = new List<Point2f>();
        var leftPts   = new List<Point2f>();
        var rightPts  = new List<Point2f>();

        foreach (var p in contour)
        {
            // horizontal bands
            if (Math.Abs(p.Y - minY) <= bandThickness)
                topPts.Add(p);
            if (Math.Abs(p.Y - maxY) <= bandThickness)
                bottomPts.Add(p);

            // vertical bands
            if (Math.Abs(p.X - minX) <= bandThickness)
                leftPts.Add(p);
            if (Math.Abs(p.X - maxX) <= bandThickness)
                rightPts.Add(p);
        }

        // --- local helpers inside the same method ---

        Vec4f FitLineFromPoints(List<Point2f> pts)
        {
            if (pts == null || pts.Count < 2)
                return new Vec4f(0, 0, 0, 0);

            using var ptsMat = new Mat(pts.Count, 1, MatType.CV_32FC2);
            for (int i = 0; i < pts.Count; i++)
                ptsMat.Set(i, 0, pts[i]);

            // OpenCvSharp version that RETURNS a Vec4f
            return Cv2.FitLine(
                ptsMat,
                DistanceTypes.L2,
                0,
                0.01,
                0.01);
        }

        void DrawFittedLine(Mat image, Vec4f line)
        {
            float vx = line.Item0;
            float vy = line.Item1;
            float x0 = line.Item2;
            float y0 = line.Item3;

            float len = (float)Math.Sqrt(vx * vx + vy * vy);
            if (len < 1e-6f) return;

            vx /= len;
            vy /= len;

            // take a long segment in both directions
            float L = Math.Max(image.Cols, image.Rows) * 2f;
            var p1 = new Point(
                (int)Math.Round(x0 - vx * L),
                (int)Math.Round(y0 - vy * L));
            var p2 = new Point(
                (int)Math.Round(x0 + vx * L),
                (int)Math.Round(y0 + vy * L));

            // clip to image bounds so we don't draw outside
            var size = new OpenCvSharp.Size(image.Cols, image.Rows);
            if (Cv2.ClipLine(size, ref p1, ref p2))
            {
                Cv2.Line(image, p1, p2, color, thickness);
            }
        }

        // --- 3. Fit lines for each side ---
        var topLine    = FitLineFromPoints(topPts);
        var bottomLine = FitLineFromPoints(bottomPts);
        var leftLine   = FitLineFromPoints(leftPts);
        var rightLine  = FitLineFromPoints(rightPts);

        // --- 4. Draw them (skip zero / invalid ones) ---
        if (topLine.Item0 != 0 || topLine.Item1 != 0)       DrawFittedLine(img, topLine);
        if (bottomLine.Item0 != 0 || bottomLine.Item1 != 0) DrawFittedLine(img, bottomLine);
        if (leftLine.Item0 != 0 || leftLine.Item1 != 0)     DrawFittedLine(img, leftLine);
        if (rightLine.Item0 != 0 || rightLine.Item1 != 0)   DrawFittedLine(img, rightLine);

        return (topLine, rightLine, bottomLine, leftLine);
    }
}