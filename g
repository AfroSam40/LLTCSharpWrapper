using System;
using System.Collections.Generic;
using OpenCvSharp;

public static class RectMetrics
{
    public static (double width, double height) GetWidthHeightFromSegments(List<(Point2f a, Point2f b)> segs)
    {
        if (segs == null || segs.Count != 4)
            throw new ArgumentException("Expected 4 segments: TL->TR, TR->BR, BR->BL, BL->TL");

        double top    = Dist(segs[0].a, segs[0].b); // TL->TR
        double right  = Dist(segs[1].a, segs[1].b); // TR->BR
        double bottom = Dist(segs[2].a, segs[2].b); // BR->BL
        double left   = Dist(segs[3].a, segs[3].b); // BL->TL

        // robust estimate: average opposite sides
        double width  = 0.5 * (top + bottom);
        double height = 0.5 * (left + right);

        // optional: enforce width >= height if you want consistent orientation
        // if (width < height) (width, height) = (height, width);

        return (width, height);
    }

    private static double Dist(Point2f p, Point2f q)
    {
        double dx = p.X - q.X;
        double dy = p.Y - q.Y;
        return Math.Sqrt(dx * dx + dy * dy);
    }
}