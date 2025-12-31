using System;
using System.Collections.Generic;
using System.Drawing;
using System.Linq;

public static class BestFitRectSegments
{
    /// <summary>
    /// Fits a best-fit rectangle to a rectangular-ish contour and returns its 4 edges as segments:
    /// (TL->TR), (TR->BR), (BR->BL), (BL->TL).
    ///
    /// contour: list of points on the contour
    /// trimQuantile: trims outliers when estimating side extremes (0..0.49)
    /// iterations: refit+reassign iterations (>=1)
    /// </summary>
    public static List<(PointF a, PointF b)> FitBestRectangleSegments(
        IReadOnlyList<PointF> contour,
        double trimQuantile = 0.02,
        int iterations = 2)
    {
        if (contour == null || contour.Count < 4)
            throw new ArgumentException("Contour must have at least 4 points.");
        trimQuantile = Math.Clamp(trimQuantile, 0.0, 0.49);
        iterations = Math.Max(1, iterations);

        int n = contour.Count;

        // centroid
        var c = Mean(contour); // double[2]

        // PCA axes u (principal) and v (perpendicular)
        var (u, v) = PcaAxes2D(contour, c); // each double[2]

        // project to PCA frame
        double[] s = new double[n];
        double[] t = new double[n];
        for (int i = 0; i < n; i++)
        {
            double dx = contour[i].X - c[0];
            double dy = contour[i].Y - c[1];
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

        // lines: left,right,top,bottom as double[3] = {a,b,c}, ax+by+c=0, (a,b) normalized
        double[][] lines = new double[4][];

        for (int it = 0; it < iterations; it++)
        {
            lines[0] = FitLineTLS(GetGroup(contour, labels, 0), contour, s, sMin); // left
            lines[1] = FitLineTLS(GetGroup(contour, labels, 1), contour, s, sMax); // right
            lines[2] = FitLineTLS(GetGroup(contour, labels, 2), contour, t, tMax); // top
            lines[3] = FitLineTLS(GetGroup(contour, labels, 3), contour, t, tMin); // bottom

            // reassign points to nearest line
            for (int i = 0; i < n; i++)
            {
                double x = contour[i].X, y = contour[i].Y;

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

        PointF TL = new((float)TLd[0], (float)TLd[1]);
        PointF TR = new((float)TRd[0], (float)TRd[1]);
        PointF BR = new((float)BRd[0], (float)BRd[1]);
        PointF BL = new((float)BLd[0], (float)BLd[1]);

        return new List<(PointF a, PointF b)>
        {
            (TL, TR),
            (TR, BR),
            (BR, BL),
            (BL, TL)
        };
    }

    // ---------------- helpers ----------------

    private static double[] Mean(IReadOnlyList<PointF> pts)
    {
        double sx = 0, sy = 0;
        for (int i = 0; i < pts.Count; i++) { sx += pts[i].X; sy += pts[i].Y; }
        return new[] { sx / pts.Count, sy / pts.Count };
    }

    private static (double[] u, double[] v) PcaAxes2D(IReadOnlyList<PointF> pts, double[] c)
    {
        double sxx = 0, sxy = 0, syy = 0;
        int n = pts.Count;

        for (int i = 0; i < n; i++)
        {
            double dx = pts[i].X - c[0];
            double dy = pts[i].Y - c[1];
            sxx += dx * dx;
            sxy += dx * dy;
            syy += dy * dy;
        }

        double inv = 1.0 / Math.Max(1, n - 1);
        sxx *= inv; sxy *= inv; syy *= inv;

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

        double vx = -uy, vy = ux; // perpendicular
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

    private static List<PointF> GetGroup(IReadOnlyList<PointF> pts, int[] labels, int side)
    {
        var g = new List<PointF>();
        for (int i = 0; i < pts.Count; i++)
            if (labels[i] == side)
                g.Add(pts[i]);
        return g;
    }

    private static double[] FitLineTLS(List<PointF> group, IReadOnlyList<PointF> allPts, double[] coord, double extreme)
    {
        // fallback: closest points to expected extreme
        if (group.Count < 10)
        {
            int n = allPts.Count;
            int take = Math.Max(10, n / 20);
            var idx = Enumerable.Range(0, n)
                .OrderBy(i => Math.Abs(coord[i] - extreme))
                .Take(take)
                .ToArray();

            group = idx.Select(i => allPts[i]).ToList();
        }

        // mean
        double mx = 0, my = 0;
        for (int i = 0; i < group.Count; i++) { mx += group[i].X; my += group[i].Y; }
        mx /= group.Count; my /= group.Count;

        // covariance
        double sxx = 0, sxy = 0, syy = 0;
        for (int i = 0; i < group.Count; i++)
        {
            double dx = group[i].X - mx;
            double dy = group[i].Y - my;
            sxx += dx * dx;
            sxy += dx * dy;
            syy += dy * dy;
        }

        double inv = 1.0 / Math.Max(1, group.Count - 1);
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
        => Math.Abs(line[0] * x + line[1] * y + line[2]);

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