using System;
using System.Collections.Generic;
using OpenCvSharp;

public static class DogboneRectHelper
{
    /// <summary>
    /// From a contour enclosing a mostly rectangular hole (with a "mouse ear" on one corner),
    /// fit one line to each side and draw crosses at the four corner intersections.
    /// Assumes the rectangle is roughly axis-aligned in the image.
    /// </summary>
    /// <param name="img">Image to draw crosses on (8UC1 or 8UC3).</param>
    /// <param name="contour">Contour of the hole (e.g. from FindContours).</param>
    /// <param name="corners">Output 4 corners in order [TL, TR, BR, BL].</param>
    /// <param name="maxSideDistance">
    /// Max distance (in pixels) from the bounding-box edges to consider a contour point
    /// as belonging to that side. Ear points that bulge inwards are mostly ignored.
    /// </param>
    /// <param name="crossHalfSize">Half-length of the cross arms (in pixels).</param>
    /// <param name="thickness">Line thickness for cross drawing.</param>
    /// <returns>true if we fitted 4 sides and produced corners; false otherwise.</returns>
    public static bool FitRectSidesAndDrawCornerCrosses(
        Mat img,
        Point[] contour,
        out Point2f[] corners,
        double maxSideDistance = 5.0,
        int crossHalfSize = 8,
        int thickness = 2)
    {
        if (img == null) throw new ArgumentNullException(nameof(img));
        if (contour == null || contour.Length < 4)
        {
            corners = Array.Empty<Point2f>();
            return false;
        }

        // 1) Axis-aligned bounding box of the contour
        Rect rect = Cv2.BoundingRect(contour);
        double leftX   = rect.X;
        double rightX  = rect.X + rect.Width;
        double topY    = rect.Y;
        double bottomY = rect.Y + rect.Height;

        // 2) Split contour points into four side-sets
        var leftPts   = new List<Point2f>();
        var rightPts  = new List<Point2f>();
        var topPts    = new List<Point2f>();
        var bottomPts = new List<Point2f>();

        foreach (var p in contour)
        {
            double x = p.X;
            double y = p.Y;

            double dxL = Math.Abs(x - leftX);
            double dxR = Math.Abs(x - rightX);
            double dyT = Math.Abs(y - topY);
            double dyB = Math.Abs(y - bottomY);

            double minDist = Math.Min(Math.Min(dxL, dxR), Math.Min(dyT, dyB));
            if (minDist > maxSideDistance)
                continue; // too far from any side, ignore (this includes most of the dogbone arc)

            // Assign to the closest side (ties broken in this order)
            if (minDist == dxL)
                leftPts.Add(p);
            else if (minDist == dxR)
                rightPts.Add(p);
            else if (minDist == dyT)
                topPts.Add(p);
            else
                bottomPts.Add(p);
        }

        // Require at least a few points for each side
        if (leftPts.Count < 2 || rightPts.Count < 2 ||
            topPts.Count < 2  || bottomPts.Count < 2)
        {
            corners = Array.Empty<Point2f>();
            return false;
        }

        // 3) Fit line to each side
        Vec4f leftLine   = FitLineFromPoints(leftPts);
        Vec4f rightLine  = FitLineFromPoints(rightPts);
        Vec4f topLine    = FitLineFromPoints(topPts);
        Vec4f bottomLine = FitLineFromPoints(bottomPts);

        // If any fit failed, bail
        if (!IsValidLine(leftLine) || !IsValidLine(rightLine) ||
            !IsValidLine(topLine)  || !IsValidLine(bottomLine))
        {
            corners = Array.Empty<Point2f>();
            return false;
        }

        // 4) Intersections -> corners
        bool okTL = TryIntersectLines(topLine,    leftLine,   out Point2f tl);
        bool okTR = TryIntersectLines(topLine,    rightLine,  out Point2f tr);
        bool okBR = TryIntersectLines(bottomLine, rightLine,  out Point2f br);
        bool okBL = TryIntersectLines(bottomLine, leftLine,   out Point2f bl);

        if (!okTL || !okTR || !okBR || !okBL)
        {
            corners = Array.Empty<Point2f>();
            return false;
        }

        corners = new[] { tl, tr, br, bl };

        // 5) Draw crosses at corners
        Scalar crossColor = img.Channels() == 1
            ? new Scalar(255)       // white on grayscale
            : new Scalar(0, 0, 255); // red on BGR

        foreach (var c in corners)
        {
            var center = new Point((int)Math.Round(c.X), (int)Math.Round(c.Y));

            // horizontal
            Cv2.Line(
                img,
                new Point(center.X - crossHalfSize, center.Y),
                new Point(center.X + crossHalfSize, center.Y),
                crossColor,
                thickness);

            // vertical
            Cv2.Line(
                img,
                new Point(center.X, center.Y - crossHalfSize),
                new Point(center.X, center.Y + crossHalfSize),
                crossColor,
                thickness);
        }

        return true;
    }

    // --- helpers ---

    private static Vec4f FitLineFromPoints(List<Point2f> pts)
    {
        // OpenCvSharp FitLine result: (vx, vy, x0, y0)
        Cv2.FitLine(
            pts,
            out Vec4f line,
            DistanceTypes.L2,
            0,
            0.01,
            0.01);

        return line;
    }

    private static bool IsValidLine(Vec4f line)
    {
        float vx = line[0], vy = line[1];
        return Math.Abs(vx) > 1e-6 || Math.Abs(vy) > 1e-6;
    }

    /// <summary>
    /// Intersect two 2D lines in FitLine form (vx,vy,x0,y0).
    /// Returns false if lines are nearly parallel.
    /// </summary>
    private static bool TryIntersectLines(Vec4f l1, Vec4f l2, out Point2f pt)
    {
        float vx1 = l1[0], vy1 = l1[1], x1 = l1[2], y1 = l1[3];
        float vx2 = l2[0], vy2 = l2[1], x2 = l2[2], y2 = l2[3];

        // normal (a,b) is perpendicular to direction (vx,vy)
        float a1 = -vy1, b1 = vx1, c1 = -(a1 * x1 + b1 * y1);
        float a2 = -vy2, b2 = vx2, c2 = -(a2 * x2 + b2 * y2);

        float det = a1 * b2 - a2 * b1;
        if (Math.Abs(det) < 1e-6f)
        {
            pt = new Point2f(float.NaN, float.NaN);
            return false; // almost parallel
        }

        float x = (b1 * c2 - b2 * c1) / det;
        float y = (c1 * a2 - c2 * a1) / det;
        pt = new Point2f(x, y);
        return true;
    }
}