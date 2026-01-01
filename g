using System;
using System.Collections.Generic;
using System.Drawing;

public static class CircleHullHelpers
{
    /// <summary>
    /// Least-squares best-fit circle to a set of 2D points.
    /// Uses algebraic fit: x² + y² + D x + E y + F = 0.
    /// Center = (-D/2, -E/2), radius = sqrt(D² + E² - 4F) / 2.
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

        double sumX = 0, sumY = 0, sumX2 = 0, sumY2 = 0, sumXY = 0;
        double sumB = 0, sumXB = 0, sumYB = 0;
        int n = points.Count;

        foreach (var pt in points)
        {
            double x = pt.X;
            double y = pt.Y;
            double r2 = x * x + y * y;   // x² + y²
            double b = -r2;

            sumX  += x;
            sumY  += y;
            sumX2 += x * x;
            sumY2 += y * y;
            sumXY += x * y;

            sumB  += b;
            sumXB += x * b;
            sumYB += y * b;
        }

        // AᵀA
        double a11 = sumX2;
        double a12 = sumXY;
        double a13 = sumX;
        double a21 = sumXY;
        double a22 = sumY2;
        double a23 = sumY;
        double a31 = sumX;
        double a32 = sumY;
        double a33 = n;

        // Aᵀb
        double b1 = sumXB;
        double b2 = sumYB;
        double b3 = sumB;

        double detA =
            a11 * (a22 * a33 - a23 * a32) -
            a12 * (a21 * a33 - a23 * a31) +
            a13 * (a21 * a32 - a22 * a31);

        if (Math.Abs(detA) < 1e-12)
            return false; // degenerate

        double detA1 =
            b1  * (a22 * a33 - a23 * a32) -
            a12 * (b2  * a33 - a23 * b3 ) +
            a13 * (b2  * a32 - a22 * b3 );

        double detA2 =
            a11 * (b2  * a33 - a23 * b3 ) -
            b1  * (a21 * a33 - a23 * a31) +
            a13 * (a21 * b3  - b2  * a31);

        double detA3 =
            a11 * (a22 * b3  - b2  * a32) -
            a12 * (a21 * b3  - b2  * a31) +
            b1  * (a21 * a32 - a22 * a31);

        double D = detA1 / detA;
        double E = detA2 / detA;
        double F = detA3 / detA;

        double cx = -D / 2.0;
        double cy = -E / 2.0;
        double radSq = cx * cx + cy * cy - F;

        if (radSq <= 0)
            return false;

        center = new PointF((float)cx, (float)cy);
        radius = (float)Math.Sqrt(radSq);
        return true;
    }

    /// <summary>
    /// "ComputeConvexHull" replacement that actually approximates a
    /// best-fit circle boundary as a polygon (so existing code that
    /// expects a hull still works).
    ///
    /// points  - slice points in 2D (e.g., projected (u,v)).
    /// hull    - polygon approximating the circle (in same coords).
    /// segments- number of vertices around the circle.
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

        if (segments < 8) segments = 8; // avoid silly polygons

        hull.Capacity = segments;

        // Build a polygon approximating the fitted circle
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