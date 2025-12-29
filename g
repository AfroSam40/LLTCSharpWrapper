using OpenCvSharp;
using System;
using System.Collections.Generic;

public static class ContourLineApprox
{
    // Returns line segments as (A,B) points in image coords
    public static List<(Point A, Point B)> ApproximateLinesFromContour(Point[] contour, double epsilonFrac = 0.01)
    {
        if (contour == null || contour.Length < 5)
            return new List<(Point, Point)>();

        double peri = Cv2.ArcLength(contour, true);
        double eps = epsilonFrac * peri;

        Point[] poly = Cv2.ApproxPolyDP(contour, eps, true);

        var lines = new List<(Point A, Point B)>(poly.Length);

        for (int i = 0; i < poly.Length; i++)
        {
            Point a = poly[i];
            Point b = poly[(i + 1) % poly.Length];
            lines.Add((a, b));
        }

        return lines;
    }
}