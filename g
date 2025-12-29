using OpenCvSharp;
using System;
using System.Linq;

public static class MouseEarCornerFinder
{
    /// <summary>
    /// Given 4 rectangle corners and a grayscale image, finds the corner opposite the mouse-ear corner.
    /// Mouse-ear corner is defined as the corner whose centered probe-rect contains the least white pixels.
    /// Returns the opposite corner (diagonally across), found as the farthest corner from the ear corner.
    /// </summary>
    /// <param name="gray">Grayscale image (8-bit recommended). White = foreground.</param>
    /// <param name="corners">Four corners (preferably TL,TR,BR,BL). If not ordered, it still works for "opposite" via farthest distance.</param>
    /// <param name="probeW">Probe rect width (pixels) in rectangle-local X direction.</param>
    /// <param name="probeH">Probe rect height (pixels) in rectangle-local Y direction.</param>
    /// <param name="whiteThreshold">
    /// If null: uses Otsu threshold to binarize. If set: uses that fixed threshold (0..255).
    /// </param>
    public static Point2f FindCornerOppositeMouseEar(
        Mat gray,
        Point2f[] corners,
        int probeW,
        int probeH,
        byte? whiteThreshold = null)
    {
        if (gray == null || gray.Empty())
            throw new ArgumentException("gray is null/empty.");
        if (gray.Type() != MatType.CV_8UC1)
            throw new ArgumentException("gray must be CV_8UC1 (8-bit single-channel).");
        if (corners == null || corners.Length != 4)
            throw new ArgumentException("corners must contain exactly 4 points.");
        if (probeW < 3 || probeH < 3)
            throw new ArgumentException("probeW/probeH too small.");

        // 1) Binarize (white = foreground)
        using var bin = new Mat();
        if (whiteThreshold.HasValue)
        {
            Cv2.Threshold(gray, bin, whiteThreshold.Value, 255, ThresholdTypes.Binary);
        }
        else
        {
            Cv2.Threshold(gray, bin, 0, 255, ThresholdTypes.Binary | ThresholdTypes.Otsu);
        }

        // 2) Compute rectangle axis directions from corners.
        // Assumption: corners are roughly TL,TR,BR,BL.
        // If your input may be in random order, order them first using your ordering function.
        // We'll still compute axes from the "best" pairings:
        // - u (x-axis): longest-ish top/bottom edge direction
        // - v (y-axis): perpendicular-ish direction using another edge
        // If you *do* have TL,TR,BR,BL, set u = TR-TL and v = BL-TL (best).
        Point2f tl = corners[0], tr = corners[1], br = corners[2], bl = corners[3];

        var u = Normalize(tr - tl); // local X (width direction)
        var v = Normalize(bl - tl); // local Y (height direction)

        // If axes are degenerate (bad ordering), fall back to using minAreaRect angle
        if (Length(u) < 1e-4f || Length(v) < 1e-4f)
        {
            // crude fallback: take two farthest corners as diagonal, derive axes using others
            // (keeps function from crashing; ordering is still recommended)
            var (a, b) = FarthestPair(corners);
            u = Normalize(b - a);
            v = new Point2f(-u.Y, u.X); // perpendicular
        }

        // 3) For each corner: extract an oriented probe patch centered on that corner and count white pixels
        int bestIdx = 0;
        int bestWhiteCount = int.MaxValue;

        for (int i = 0; i < 4; i++)
        {
            int whiteCount = CountWhiteInOrientedPatch(bin, corners[i], u, v, probeW, probeH);
            if (whiteCount < bestWhiteCount)
            {
                bestWhiteCount = whiteCount;
                bestIdx = i;
            }
        }

        // 4) Mouse-ear corner = corners[bestIdx]
        var earCorner = corners[bestIdx];

        // 5) Opposite corner: the farthest of the other 3 corners
        Point2f opposite = corners.Where((p, idx) => idx != bestIdx)
                                  .OrderByDescending(p => Dist2(p, earCorner))
                                  .First();

        return opposite;
    }

    // ---- helpers ----

    private static int CountWhiteInOrientedPatch(Mat bin, Point2f corner, Point2f u, Point2f v, int w, int h)
    {
        // Build affine transform that maps destination patch coords -> source image coords.
        // For patch pixel (x,y), local dx = x - w/2, dy = y - h/2.
        // src = corner + u*dx + v*dy
        float halfW = (w - 1) * 0.5f;
        float halfH = (h - 1) * 0.5f;

        float m00 = u.X;
        float m01 = v.X;
        float m02 = corner.X - u.X * halfW - v.X * halfH;

        float m10 = u.Y;
        float m11 = v.Y;
        float m12 = corner.Y - u.Y * halfW - v.Y * halfH;

        using var M = new Mat(2, 3, MatType.CV_32F);
        M.Set(0, 0, m00); M.Set(0, 1, m01); M.Set(0, 2, m02);
        M.Set(1, 0, m10); M.Set(1, 1, m11); M.Set(1, 2, m12);

        using var patch = new Mat();
        // We want M to be interpreted as dst->src, so set WARP_INVERSE_MAP.
        Cv2.WarpAffine(
            bin, patch, M, new Size(w, h),
            InterpolationFlags.Nearest,
            BorderTypes.Constant,
            Scalar.Black,
            flags: WarpAffineFlags.WarpInverseMap);

        return Cv2.CountNonZero(patch);
    }

    private static float Dist2(Point2f a, Point2f b)
    {
        float dx = a.X - b.X;
        float dy = a.Y - b.Y;
        return dx * dx + dy * dy;
    }

    private static float Length(Point2f p) => (float)Math.Sqrt(p.X * p.X + p.Y * p.Y);

    private static Point2f Normalize(Point2f p)
    {
        float len = Length(p);
        return len > 1e-6f ? new Point2f(p.X / len, p.Y / len) : new Point2f(0, 0);
    }

    private static (Point2f A, Point2f B) FarthestPair(Point2f[] pts)
    {
        float best = -1;
        Point2f a = pts[0], b = pts[1];
        for (int i = 0; i < pts.Length; i++)
        for (int j = i + 1; j < pts.Length; j++)
        {
            float d = Dist2(pts[i], pts[j]);
            if (d > best) { best = d; a = pts[i]; b = pts[j]; }
        }
        return (a, b);
    }
}