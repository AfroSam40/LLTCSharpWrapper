using OpenCvSharp;
using System;
using System.Linq;

public static class RectWallsExpectedSize
{
    // Returns 4 wall segments: left, right, top, bottom
    // Also outputs the filtered (ear-removed) inlier points if you want to debug / refit.
    public static (Point2f A, Point2f B)[] FindWallsAndFilterEar(
        Point[] contour,
        float expectedW,
        float expectedH,
        float edgeTolPx = 3.0f,          // how close a point must be to an edge to count as inlier
        float alongTolPx = 6.0f,         // how far outside [top,bottom]/[left,right] we allow for edge membership
        float centerSearchFrac = 0.05f,  // search +/- this fraction of W/H around initial center
        int searchSteps = 11,            // odd number is nice (includes 0)
        out Point2f[] inliersOriginal)
    {
        if (contour == null || contour.Length < 20)
            throw new ArgumentException("Contour too small.");

        // ---------- 1) PCA orientation ----------
        // Build Nx2 float matrix
        using var ptsMat = new Mat(contour.Length, 2, MatType.CV_32F);
        for (int i = 0; i < contour.Length; i++)
        {
            ptsMat.Set(i, 0, (float)contour[i].X);
            ptsMat.Set(i, 1, (float)contour[i].Y);
        }

        using var mean = new Mat();
        using var evecs = new Mat();
        Cv2.PCACompute(ptsMat, mean, evecs);

        float cx = mean.At<float>(0, 0);
        float cy = mean.At<float>(0, 1);

        float vx = evecs.At<float>(0, 0);
        float vy = evecs.At<float>(0, 1);
        double angle = Math.Atan2(vy, vx);

        // Rotate by -angle to align dominant axis with X
        double ca = Math.Cos(-angle), sa = Math.Sin(-angle);

        Point2f[] rotPts = new Point2f[contour.Length];
        for (int i = 0; i < contour.Length; i++)
        {
            float x = contour[i].X - cx;
            float y = contour[i].Y - cy;
            float xr = (float)(x * ca - y * sa);
            float yr = (float)(x * sa + y * ca);
            rotPts[i] = new Point2f(xr, yr);
        }

        // ---------- 2) Initial center estimate (median is robust to the ear) ----------
        float medX = Median(rotPts.Select(p => p.X).ToArray());
        float medY = Median(rotPts.Select(p => p.Y).ToArray());

        // ---------- 3) Grid search for best center (max inliers near expected edges) ----------
        float dxMax = expectedW * centerSearchFrac;
        float dyMax = expectedH * centerSearchFrac;

        int bestCount = -1;
        float bestCxR = medX, bestCyR = medY;

        for (int ix = 0; ix < searchSteps; ix++)
        {
            float tx = Lerp(-dxMax, dxMax, ix / (float)(searchSteps - 1));
            for (int iy = 0; iy < searchSteps; iy++)
            {
                float ty = Lerp(-dyMax, dyMax, iy / (float)(searchSteps - 1));
                float cxr = medX + tx;
                float cyr = medY + ty;

                int count = CountEdgeInliers(rotPts, cxr, cyr, expectedW, expectedH, edgeTolPx, alongTolPx);
                if (count > bestCount)
                {
                    bestCount = count;
                    bestCxR = cxr;
                    bestCyR = cyr;
                }
            }
        }

        // ---------- 4) Define rectangle edges in rotated space ----------
        float left   = bestCxR - expectedW * 0.5f;
        float right  = bestCxR + expectedW * 0.5f;
        float top    = bestCyR - expectedH * 0.5f;
        float bottom = bestCyR + expectedH * 0.5f;

        // ---------- 5) Filter ear: keep only points close to any edge ----------
        var inliersR = rotPts.Where(p => IsNearAnyEdge(p, left, right, top, bottom, edgeTolPx, alongTolPx))
                             .ToArray();

        // ---------- 6) Build wall segments in rotated space ----------
        var leftSegR   = (new Point2f(left,  top),    new Point2f(left,  bottom));
        var rightSegR  = (new Point2f(right, top),    new Point2f(right, bottom));
        var topSegR    = (new Point2f(left,  top),    new Point2f(right, top));
        var bottomSegR = (new Point2f(left,  bottom), new Point2f(right, bottom));

        // ---------- 7) Rotate back to original space ----------
        double cb = Math.Cos(angle), sb = Math.Sin(angle);

        Point2f Unrotate(Point2f p)
        {
            // p is in rotated coords centered at (bestCxR, bestCyR)??? No:
            // our rotated coords are centered at PCA mean (cx,cy) but not translated by best center.
            // So we just unrotate and add back (cx,cy).
            float x = p.X;
            float y = p.Y;
            float xo = (float)(x * cb - y * sb) + cx;
            float yo = (float)(x * sb + y * cb) + cy;
            return new Point2f(xo, yo);
        }

        inliersOriginal = inliersR.Select(Unrotate).ToArray();

        (Point2f, Point2f) Back((Point2f A, Point2f B) s) => (Unrotate(s.A), Unrotate(s.B));

        return new[]
        {
            Back(leftSegR),
            Back(rightSegR),
            Back(topSegR),
            Back(bottomSegR),
        };
    }

    // --- helpers ---
    private static int CountEdgeInliers(Point2f[] pts, float cX, float cY, float w, float h, float edgeTol, float alongTol)
    {
        float left = cX - w * 0.5f, right = cX + w * 0.5f;
        float top = cY - h * 0.5f, bottom = cY + h * 0.5f;

        int count = 0;
        for (int i = 0; i < pts.Length; i++)
            if (IsNearAnyEdge(pts[i], left, right, top, bottom, edgeTol, alongTol))
                count++;
        return count;
    }

    private static bool IsNearAnyEdge(Point2f p, float left, float right, float top, float bottom, float edgeTol, float alongTol)
    {
        // close to vertical edges AND within y-range (with slack)
        bool nearLeft  = Math.Abs(p.X - left)  <= edgeTol && p.Y >= top - alongTol && p.Y <= bottom + alongTol;
        bool nearRight = Math.Abs(p.X - right) <= edgeTol && p.Y >= top - alongTol && p.Y <= bottom + alongTol;

        // close to horizontal edges AND within x-range (with slack)
        bool nearTop    = Math.Abs(p.Y - top)    <= edgeTol && p.X >= left - alongTol && p.X <= right + alongTol;
        bool nearBottom = Math.Abs(p.Y - bottom) <= edgeTol && p.X >= left - alongTol && p.X <= right + alongTol;

        return nearLeft || nearRight || nearTop || nearBottom;
    }

    private static float Median(float[] a)
    {
        var s = a.OrderBy(v => v).ToArray();
        int n = s.Length;
        if (n == 0) return 0;
        return (n % 2 == 1) ? s[n / 2] : 0.5f * (s[n / 2 - 1] + s[n / 2]);
    }

    private static float Lerp(float a, float b, float t) => a + (b - a) * t;
}

Point2f[] inliers;
var walls = RectWallsExpectedSize.FindWallsAndFilterEar(
    contour,
    expectedW: 200, expectedH: 120,
    edgeTolPx: 3, alongTolPx: 6,
    out inliers
);

// walls[0]=left, [1]=right, [2]=top, [3]=bottom
Cv2.Line(img, (Point)walls[0].A, (Point)walls[0].B, Scalar.Lime, 2);
Cv2.Line(img, (Point)walls[1].A, (Point)walls[1].B, Scalar.Lime, 2);
Cv2.Line(img, (Point)walls[2].A, (Point)walls[2].B, Scalar.Lime, 2);
Cv2.Line(img, (Point)walls[3].A, (Point)walls[3].B, Scalar.Lime, 2);