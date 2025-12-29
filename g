using OpenCvSharp;
using System;

public static class SimpleMouseEar
{
    /// <summary>
    /// Crops an axis-aligned patch centered at each corner, computes mean intensity,
    /// finds the corner with the lowest mean (mouse ear), and returns the opposite corner.
    /// </summary>
    /// <param name="gray">CV_8UC1 grayscale image</param>
    /// <param name="corners">4 corner points (any order)</param>
    /// <param name="patchW">crop width in pixels</param>
    /// <param name="patchH">crop height in pixels</param>
    public static Point2f CornerOppositeMouseEar_ByMeanCrop(
        Mat gray,
        Point2f[] corners,
        int patchW,
        int patchH,
        out int mouseEarIndex,
        out double[] means)
    {
        if (gray == null || gray.Empty())
            throw new ArgumentException("gray is null/empty");
        if (gray.Type() != MatType.CV_8UC1)
            throw new ArgumentException("gray must be CV_8UC1");
        if (corners == null || corners.Length != 4)
            throw new ArgumentException("corners must have length 4");
        if (patchW < 3 || patchH < 3)
            throw new ArgumentException("patchW/patchH too small");

        means = new double[4];

        int bestIdx = 0;
        double bestMean = double.MaxValue;

        for (int i = 0; i < 4; i++)
        {
            var r = CenteredRectClamped(corners[i], patchW, patchH, gray.Width, gray.Height);
            using var roi = new Mat(gray, r);
            double m = Cv2.Mean(roi).Val0;   // grayscale mean
            means[i] = m;

            if (m < bestMean)
            {
                bestMean = m;
                bestIdx = i;
            }
        }

        mouseEarIndex = bestIdx;

        // opposite corner = farthest of the other 3 (works regardless of ordering)
        Point2f ear = corners[bestIdx];
        int oppIdx = -1;
        double bestDist2 = -1;

        for (int i = 0; i < 4; i++)
        {
            if (i == bestIdx) continue;
            double dx = corners[i].X - ear.X;
            double dy = corners[i].Y - ear.Y;
            double d2 = dx * dx + dy * dy;
            if (d2 > bestDist2)
            {
                bestDist2 = d2;
                oppIdx = i;
            }
        }

        return corners[oppIdx];
    }

    private static Rect CenteredRectClamped(Point2f c, int w, int h, int imgW, int imgH)
    {
        int x = (int)Math.Round(c.X - w / 2.0);
        int y = (int)Math.Round(c.Y - h / 2.0);

        // clamp top-left
        x = Math.Max(0, Math.Min(x, imgW - 1));
        y = Math.Max(0, Math.Min(y, imgH - 1));

        // clamp size so ROI stays inside
        int ww = Math.Min(w, imgW - x);
        int hh = Math.Min(h, imgH - y);

        // also ensure >=1
        ww = Math.Max(1, ww);
        hh = Math.Max(1, hh);

        return new Rect(x, y, ww, hh);
    }
}

int earIdx;
double[] means;
Point2f opposite = SimpleMouseEar.CornerOppositeMouseEar_ByMeanCrop(
    gray,
    corners,
    patchW: 60,
    patchH: 60,
    out earIdx,
    out means
);

Console.WriteLine($"Mouse ear corner index: {earIdx}, means: {string.Join(",", means)}");