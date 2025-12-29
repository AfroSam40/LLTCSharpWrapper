using OpenCvSharp;
using System;
using System.Linq;
using System.Collections.Generic;

public static class FixedSizeRectFit
{
    // Returns 4 wall segments: left, right, top, bottom (in image coordinates)
    public static (Point2f A, Point2f B)[] FindWallsFixedSize(
        Point[] contour,
        float expectedW,
        float expectedH,
        float edgeTolPx = 3f,          // distance to a wall to count as inlier
        float alongTolPx = 8f,         // slack past wall endpoints
        float angleSearchDeg = 20f,    // search +/- around initial angle
        int angleSteps = 41,           // number of angles to test
        float centerSearchFrac = 0.06f,// search +/- fraction of W/H for center
        int centerSteps = 11,          // grid steps for center search
        out Point2f[] inliers)         // filtered points (ear removed)
    {
        if (contour == null || contour.Length < 20)
            throw new ArgumentException("Contour too small.");

        // --- initial guess angle from min area rect (more stable than PCA for rectangles) ---
        var rr = Cv2.MinAreaRect(contour);
        float a0 = rr.Angle;

        // OpenCV angle conventions can swap width/height; normalize so angle refers to expectedW direction
        float w0 = rr.Size.Width;
        float h0 = rr.Size.Height;
        // If rr "width" corresponds more to expectedH, rotate by 90 so axis aligns better
        if (Math.Abs(w0 - expectedH) < Math.Abs(w0 - expectedW))
            a0 += 90f;

        // Use mean as rotation center for numerical stability
        var mean = new Point2f((float)contour.Average(p => p.X), (float)contour.Average(p => p.Y));

        // Preconvert to Point2f
        var pts = contour.Select(p => new Point2f(p.X, p.Y)).ToArray();

        int bestScore = int.MinValue;
        float bestAngle = a0;
        float bestCxR = 0, bestCyR = 0;   // best center in rotated coords
        Point2f[] bestInliersR = Array.Empty<Point2f>();
        Point2f[] bestRotPts = Array.Empty<Point2f>();

        // --- angle search ---
        for (int ia = 0; ia < angleSteps; ia++)
        {
            float t = angleSteps == 1 ? 0 : ia / (float)(angleSteps - 1);
            float ang = a0 + Lerp(-angleSearchDeg, angleSearchDeg, t);

            // rotate points by -ang
            var rotPts = RotatePoints(pts, mean, -ang);

            // robust initial center: median in rotated space
            float medX = Median(rotPts.Select(p => p.X).ToArray());
            float medY = Median(rotPts.Select(p => p.Y).ToArray());

            float dxMax = expectedW * centerSearchFrac;
            float dyMax = expectedH * centerSearchFrac;

            // center grid search
            for (int ix = 0; ix < centerSteps; ix++)
            {
                float cxr = medX + Lerp(-dxMax, dxMax, centerSteps == 1 ? 0 : ix / (float)(centerSteps - 1));
                for (int iy = 0; iy < centerSteps; iy++)
                {
                    float cyr = medY + Lerp(-dyMax, dyMax, centerSteps == 1 ? 0 : iy / (float)(centerSteps - 1));

                    ScoreRect(rotPts, cxr, cyr, expectedW, expectedH, edgeTolPx, alongTolPx,
                              out int inlierCount, out int outsideCount, out Point2f[] inl);

                    // score: reward inliers, penalize points far from the rectangle (ear tends to increase outsideCount)
                    int score = inlierCount - 2 * outsideCount;

                    if (score > bestScore)
                    {
                        bestScore = score;
                        bestAngle = ang;
                        bestCxR = cxr;
                        bestCyR = cyr;
                        bestInliersR = inl;
                        bestRotPts = rotPts;
                    }
                }
            }
        }

        // Build best rectangle walls in rotated space
        float left   = bestCxR - expectedW * 0.5f;
        float right  = bestCxR + expectedW * 0.5f;
        float top    = bestCyR - expectedH * 0.5f;
        float bottom = bestCyR + expectedH * 0.5f;

        var leftSegR   = (new Point2f(left,  top),    new Point2f(left,  bottom));
        var rightSegR  = (new Point2f(right, top),    new Point2f(right, bottom));
        var topSegR    = (new Point2f(left,  top),    new Point2f(right, top));
        var bottomSegR = (new Point2f(left,  bottom), new Point2f(right, bottom));

        // Rotate back to image coords by +bestAngle
        Point2f Unrotate(Point2f pr) => RotatePoint(pr, new Point2f(0,0), bestAngle); // angle only
        // But our rotated points were around 'mean'. So undo properly:
        Point2f Back(Point2f pr)
        {
            // pr is in rotated frame centered at mean (because RotatePoints did that),
            // so rotate back around origin then translate back.
            var p0 = new Point2f(pr.X, pr.Y);
            var p1 = RotatePoint(p0, new Point2f(0,0), bestAngle);
            return new Point2f(p1.X + mean.X, p1.Y + mean.Y);
        }

        inliers = bestInliersR.Select(Back).ToArray();

        return new[]
        {
            (Back(leftSegR.Item1),   Back(leftSegR.Item2)),   // left
            (Back(rightSegR.Item1),  Back(rightSegR.Item2)),  // right
            (Back(topSegR.Item1),    Back(topSegR.Item2)),    // top
            (Back(bottomSegR.Item1), Back(bottomSegR.Item2)), // bottom
        };
    }

    // --- scoring: count points near any of the 4 walls; also count points clearly outside the rectangle ---
    private static void ScoreRect(Point2f[] rotPts, float cx, float cy, float w, float h, float edgeTol, float alongTol,
                                  out int inlierCount, out int outsideCount, out Point2f[] inliers)
    {
        float left = cx - w * 0.5f, right = cx + w * 0.5f;
        float top = cy - h * 0.5f, bottom = cy + h * 0.5f;

        var inl = new List<Point2f>(rotPts.Length);
        int outCnt = 0;

        foreach (var p in rotPts)
        {
            bool near =
                (Math.Abs(p.X - left)  <= edgeTol && p.Y >= top - alongTol && p.Y <= bottom + alongTol) ||
                (Math.Abs(p.X - right) <= edgeTol && p.Y >= top - alongTol && p.Y <= bottom + alongTol) ||
                (Math.Abs(p.Y - top)   <= edgeTol && p.X >= left - alongTol && p.X <= right + alongTol) ||
                (Math.Abs(p.Y - bottom)<= edgeTol && p.X >= left - alongTol && p.X <= right + alongTol);

            if (near) inl.Add(p);

            // "outside" = clearly beyond the expected rectangle by more than alongTol
            if (p.X < left - alongTol || p.X > right + alongTol || p.Y < top - alongTol || p.Y > bottom + alongTol)
                outCnt++;
        }

        inliers = inl.ToArray();
        inlierCount = inliers.Length;
        outsideCount = outCnt;
    }

    // --- geometry helpers ---
    private static Point2f[] RotatePoints(Point2f[] pts, Point2f center, float angleDeg)
    {
        double a = angleDeg * Math.PI / 180.0;
        double c = Math.Cos(a), s = Math.Sin(a);

        var outPts = new Point2f[pts.Length];
        for (int i = 0; i < pts.Length; i++)
        {
            float x = pts[i].X - center.X;
            float y = pts[i].Y - center.Y;
            float xr = (float)(x * c - y * s);
            float yr = (float)(x * s + y * c);
            outPts[i] = new Point2f(xr, yr);
        }
        return outPts;
    }

    private static Point2f RotatePoint(Point2f p, Point2f center, float angleDeg)
    {
        double a = angleDeg * Math.PI / 180.0;
        double c = Math.Cos(a), s = Math.Sin(a);
        float x = p.X - center.X;
        float y = p.Y - center.Y;
        return new Point2f((float)(x * c - y * s) + center.X, (float)(x * s + y * c) + center.Y);
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
var walls = FixedSizeRectFit.FindWallsFixedSize(
    contour,
    expectedW: 220, expectedH: 180,
    edgeTolPx: 3,
    alongTolPx: 10,
    angleSearchDeg: 30,
    angleSteps: 61,
    out inliers
);

// draw walls
Cv2.Line(img, (Point)walls[0].A, (Point)walls[0].B, Scalar.Lime, 2);
Cv2.Line(img, (Point)walls[1].A, (Point)walls[1].B, Scalar.Lime, 2);
Cv2.Line(img, (Point)walls[2].A, (Point)walls[2].B, Scalar.Lime, 2);
Cv2.Line(img, (Point)walls[3].A, (Point)walls[3].B, Scalar.Lime, 2);

// optional: visualize inliers (ear removed)
foreach (var p in inliers)
    Cv2.Circle(img, (Point)p, 1, Scalar.Cyan, -1);