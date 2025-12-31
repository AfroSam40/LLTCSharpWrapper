using System;
using System.Collections.Generic;
using System.Linq;
using OpenCvSharp;

public static class BestFitRectOpenCvSharp
{
    /// <summary>
    /// Best-fit rectangle (as 4 line segments) from a rectangular-ish contour.
    /// Returns 4 segments: (TL->TR), (TR->BR), (BR->BL), (BL->TL)
    /// </summary>
    public static List<(Point2f a, Point2f b)> FitBestRectangleSegments(
        Point[] contour,
        double trimQuantile = 0.02,
        int iterations = 2)
    {
        if (contour == null || contour.Length < 4)
            throw new ArgumentException("Contour must have at least 4 points.");

        trimQuantile = Math.Clamp(trimQuantile, 0.0, 0.49);
        iterations = Math.Max(1, iterations);

        int n = contour.Length;

        // Convert to double[][] for math
        double[][] pts = new double[n][];
        for (int i = 0; i < n; i++)
            pts[i] = new double[] { contour[i].X, contour[i].Y };

        // centroid
        double[] c = Mean(pts);

        // PCA axes u (principal) and v (perpendicular)
        (double[] u, double[] v) = PcaAxes2D(pts, c);

        // project to PCA frame
        double[] s = new double[n];
        double[] t = new double[n];
        for (int i = 0; i < n; i++)
        {
            double dx = pts[i][0] - c[0];
            double dy = pts[i][1] - c[1];
            s[i] = dx * u[0] + dy * u[1];
            t[i] = dx * v[0] + dy * v[1];
        }

        // robust extremes
        double sMin = Quantile(s, trimQuantile);
        double sMax = Quantile(s, 1.0 - trimQuantile);
        double tMin = Quantile(t, trimQuantile);
        double tMax = Quantile(t, 1.0 - trimQuantile);

        // labels: 0=left,1=right,2=top,3=bottom
        int[] labels = new int[n];
        for (int i = 0; i < n; i++)
            labels[i] = AssignInitialSide(s[i], t[i], sMin, sMax, tMin, tMax);

        // lines: left,right,top,bottom => each double[3] = {a,b,c} for ax+by+c=0, (a,b) normalized
        double[][] lines = new double[4][];

        for (int it = 0; it < iterations; it++)
        {
            lines[0] = FitLineTLS(GetGroup(pts, labels, 0), pts, s, sMin); // left
            lines[1] = FitLineTLS(GetGroup(pts, labels, 1), pts, s, sMax); // right
            lines[2] = FitLineTLS(GetGroup(pts, labels, 2), pts, t, tMax); // top
            lines[3] = FitLineTLS(GetGroup(pts, labels, 3), pts, t, tMin); // bottom

            // reassign to nearest line
            for (int i = 0; i < n; i++)
            {
                double x = pts[i][0], y = pts[i][1];

                double bestD = DistLine(lines[0], x, y);
                int bestK = 0;

                double d1 = DistLine(lines[1], x, y); if (d1 < bestD) { bestD = d1; bestK = 1; }
                double d2 = DistLine(lines[2], x, y); if (d2 < bestD) { bestD = d2; bestK = 2; }
                double d3 = DistLine(lines[3], x, y); if (d3 < bestD) { bestD = d3; bestK = 3; }

                labels[i] = bestK;
            }
        }

        // intersections -> corners (TL,TR,BR,BL)
        double[] TLd = Intersect(lines[2], lines[0]);
        double[] TRd = Intersect(lines[2], lines[1]);
        double[] BRd = Intersect(lines[3], lines[1]);
        double[] BLd = Intersect(lines[3], lines[0]);

        Point2f TL = new((float)TLd[0], (float)TLd[1]);
        Point2f TR = new((float)TRd[0], (float)TRd[1]);
        Point2f BR = new((float)BRd[0], (float)BRd[1]);
        Point2f BL = new((float)BLd[0], (float)BLd[1]);

        return new List<(Point2f a, Point2f b)>
        {
            (TL, TR),
            (TR, BR),
            (BR, BL),
            (BL, TL)
        };
    }

    // ---------------- helpers (no custom data types) ----------------

    private static double[] Mean(double[][] pts)
    {
        double sx = 0, sy = 0;
        for (int i = 0; i < pts.Length; i++) { sx += pts[i][0]; sy += pts[i][1]; }
        return new[] { sx / pts.Length, sy / pts.Length };
    }

    private static (double[] u, double[] v) PcaAxes2D(double[][] pts, double[] c)
    {
        double sxx = 0, sxy = 0, syy = 0;
        int n = pts.Length;

        for (int i = 0; i < n; i++)
        {
            double dx = pts[i][0] - c[0];
            double dy = pts[i][1] - c[1];
            sxx += dx * dx;
            sxy += dx * dy;
            syy += dy * dy;
        }

        double inv = 1.0 / Math.Max(1, n - 1);
        sxx *= inv; sxy *= inv; syy *= inv;

        // largest eigenvector of 2x2 covariance
        double tr = sxx + syy;
        double det = sxx * syy - sxy * sxy;
        double disc = Math.Sqrt(Math.Max(0.0, tr * tr - 4.0 * det));
        double lambdaBig = 0.5 * (tr + disc);

        double ux, uy;
        if (Math.Abs(sxy) > 1e-12)
        {
            ux = 1.0;
            uy = -((sxx - lambdaBig) / sxy) * ux;
        }
        else
        {
            if (sxx >= syy) { ux = 1; uy = 0; }
            else { ux = 0; uy = 1; }
        }

        double un = Math.Sqrt(ux * ux + uy * uy);
        if (un < 1e-12) { ux = 1; uy = 0; un = 1; }
        ux /= un; uy /= un;

        // v perpendicular to u
        double vx = -uy, vy = ux;
        return (new[] { ux, uy }, new[] { vx, vy });
    }

    private static int AssignInitialSide(double s, double t, double sMin, double sMax, double tMin, double tMax)
    {
        double dLeft = Math.Abs(s - sMin);
        double dRight = Math.Abs(s - sMax);
        double dTop = Math.Abs(t - tMax);
        double dBottom = Math.Abs(t - tMin);

        double best = dLeft;
        int side = 0;
        if (dRight < best) { best = dRight; side = 1; }
        if (dTop < best) { best = dTop; side = 2; }
        if (dBottom < best) { best = dBottom; side = 3; }
        return side;
    }

    private static double[][] GetGroup(double[][] pts, int[] labels, int side)
    {
        var g = new List<double[]>();
        for (int i = 0; i < pts.Length; i++)
            if (labels[i] == side)
                g.Add(pts[i]);
        return g.ToArray();
    }

    private static double[] FitLineTLS(double[][] group, double[][] allPts, double[] coord, double extreme)
    {
        // fallback: pick points closest to expected extreme
        if (group.Length < 10)
        {
            int n = allPts.Length;
            int take = Math.Max(10, n / 20);
            int[] idx = Enumerable.Range(0, n)
                                  .OrderBy(i => Math.Abs(coord[i] - extreme))
                                  .Take(take)
                                  .ToArray();
            group = idx.Select(i => allPts[i]).ToArray();
        }

        // mean
        double mx = 0, my = 0;
        for (int i = 0; i < group.Length; i++) { mx += group[i][0]; my += group[i][1]; }
        mx /= group.Length; my /= group.Length;

        // covariance
        double sxx = 0, sxy = 0, syy = 0;
        for (int i = 0; i < group.Length; i++)
        {
            double dx = group[i][0] - mx;
            double dy = group[i][1] - my;
            sxx += dx * dx;
            sxy += dx * dy;
            syy += dy * dy;
        }
        double inv = 1.0 / Math.Max(1, group.Length - 1);
        sxx *= inv; sxy *= inv; syy *= inv;

        // smallest eigenvector => normal (a,b)
        double tr = sxx + syy;
        double det = sxx * syy - sxy * sxy;
        double disc = Math.Sqrt(Math.Max(0.0, tr * tr - 4.0 * det));
        double lambdaSmall = 0.5 * (tr - disc);

        double a, b;
        if (Math.Abs(sxy) > 1e-12)
        {
            double vx = 1.0;
            double vy = -((sxx - lambdaSmall) / sxy) * vx;
            double nn = Math.Sqrt(vx * vx + vy * vy);
            if (nn < 1e-12) { vx = 1; vy = 0; nn = 1; }
            a = vx / nn;
            b = vy / nn;
        }
        else
        {
            if (sxx <= syy) { a = 1; b = 0; }
            else { a = 0; b = 1; }
        }

        double c = -(a * mx + b * my);

        // normalize
        double norm = Math.Sqrt(a * a + b * b);
        if (norm > 1e-12) { a /= norm; b /= norm; c /= norm; }

        return new[] { a, b, c };
    }

    private static double DistLine(double[] line, double x, double y)
        => Math.Abs(line[0] * x + line[1] * y + line[2]); // (a,b) normalized

    private static double[] Intersect(double[] l1, double[] l2)
    {
        double a1 = l1[0], b1 = l1[1], c1 = l1[2];
        double a2 = l2[0], b2 = l2[1], c2 = l2[2];

        double det = a1 * b2 - a2 * b1;
        if (Math.Abs(det) < 1e-12)
            throw new InvalidOperationException("Lines are parallel or nearly parallel.");

        double x = (b1 * c2 - b2 * c1) / det;
        double y = (c1 * a2 - c2 * a1) / det;
        return new[] { x, y };
    }

    private static double Quantile(double[] data, double q)
    {
        if (data == null || data.Length == 0) throw new ArgumentException("Empty data.");
        if (q <= 0) return data.Min();
        if (q >= 1) return data.Max();

        double[] a = (double[])data.Clone();
        Array.Sort(a);

        double pos = (a.Length - 1) * q;
        int i = (int)Math.Floor(pos);
        double frac = pos - i;

        return (i + 1 < a.Length) ? a[i] * (1 - frac) + a[i + 1] * frac : a[i];
    }
}