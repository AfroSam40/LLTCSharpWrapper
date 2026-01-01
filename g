using System;
using System.Collections.Generic;
using System.Drawing;

public static class CircleHullHelpers
{
    /// <summary>
    /// Robust best-fit "circle" for a set of 2D points.
    /// - Center: centroid of points
    /// - Radius: median distance from centroid (robust, no huge blow-up)
    /// Returns false if less than 3 points.
    /// </summary>
    public static bool TryFitCircle(
        List<Point> points,
        out PointF center,
        out float radius)
    {
        center = new PointF(0, 0);
        radius = 0f;

        if (points == null || points.Count < 3)
            return false;

        int n = points.Count;

        // 1) Centroid
        double sumX = 0, sumY = 0;
        foreach (var p in points)
        {
            sumX += p.X;
            sumY += p.Y;
        }

        double cx = sumX / n;
        double cy = sumY / n;
        center = new PointF((float)cx, (float)cy);

        // 2) Distances from centroid
        var dists = new double[n];
        for (int i = 0; i < n; i++)
        {
            double dx = points[i].X - cx;
            double dy = points[i].Y - cy;
            dists[i] = Math.Sqrt(dx * dx + dy * dy);
        }

        Array.Sort(dists);
        double median = (n % 2 == 1)
            ? dists[n / 2]
            : 0.5 * (dists[n / 2 - 1] + dists[n / 2]);

        // 3) Optional: measure spread and clamp if weird
        double minD = dists[0];
        double maxD = dists[n - 1];
        double spread = maxD - minD;

        // If spread is tiny (almost all points at same radius),
        // just use the average distance.
        if (spread < 1e-6)
        {
            double avg = 0;
            for (int i = 0; i < n; i++) avg += dists[i];
            avg /= n;
            radius = (float)avg;
            return true;
        }

        // Otherwise, median is fine and won't blow up.
        radius = (float)median;
        return true;
    }

    /// <summary>
    /// "Convex hull" replacement that approximates the robust best-fit circle
    /// as a polygon with 'segments' vertices.
    /// This avoids huge radii when there are only a few points.
    /// </summary>
    public static bool ComputeConvexHull(
        List<Point> points,
        out List<Point> hull,
        int segments = 64)
    {
        hull = new List<Point>();

        if (points == null || points.Count < 3)
            return false;

        if (!TryFitCircle(points, out var center, out float radius))
            return false;

        if (segments < 8) segments = 8;

        hull.Capacity = segments;

        for (int i = 0; i < segments; i++)
        {
            double theta = 2.0 * Math.PI * i / segments;
            double x = center.X + radius * Math.Cos(theta);
            double y = center.Y + radius * Math.Sin(theta);
            hull.Add(new Point((int)Math.Round(x), (int)Math.Round(y)));
        }

        return true;
    }
}