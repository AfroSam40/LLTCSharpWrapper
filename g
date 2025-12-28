// ---- Fit lines to 4 sides of `contour` and draw lines + corner crosses ----
if (contour != null && contour.Length >= 4)
{
    var pts = contour.Select(p => new OpenCvSharp.Point2f(p.X, p.Y)).ToList();

    // Bounding box
    float minX = pts.Min(p => p.X);
    float maxX = pts.Max(p => p.X);
    float minY = pts.Min(p => p.Y);
    float maxY = pts.Max(p => p.Y);

    float size   = Math.Max(maxX - minX, maxY - minY);
    float band   = 0.05f * size;   // 5% of size as side band
    int   minPts = 10;

    var bottomPts = new List<OpenCvSharp.Point2f>();
    var topPts    = new List<OpenCvSharp.Point2f>();
    var leftPts   = new List<OpenCvSharp.Point2f>();
    var rightPts  = new List<OpenCvSharp.Point2f>();

    // classify contour points to sides by proximity to bbox edges
    foreach (var p in pts)
    {
        if (Math.Abs(p.Y - maxY) <= band) bottomPts.Add(p); // bottom
        if (Math.Abs(p.Y - minY) <= band) topPts.Add(p);    // top
        if (Math.Abs(p.X - minX) <= band) leftPts.Add(p);   // left
        if (Math.Abs(p.X - maxX) <= band) rightPts.Add(p);  // right
    }

    bool hasBottom = bottomPts.Count >= minPts;
    bool hasTop    = topPts.Count    >= minPts;
    bool hasLeft   = leftPts.Count   >= minPts;
    bool hasRight  = rightPts.Count  >= minPts;

    OpenCvSharp.Line2D bottomLine = default;
    OpenCvSharp.Line2D topLine    = default;
    OpenCvSharp.Line2D leftLine   = default;
    OpenCvSharp.Line2D rightLine  = default;

    if (hasBottom)
        bottomLine = OpenCvSharp.Cv2.FitLine(bottomPts, OpenCvSharp.DistanceTypes.L2, 0, 0.01, 0.01);
    if (hasTop)
        topLine    = OpenCvSharp.Cv2.FitLine(topPts,    OpenCvSharp.DistanceTypes.L2, 0, 0.01, 0.01);
    if (hasLeft)
        leftLine   = OpenCvSharp.Cv2.FitLine(leftPts,   OpenCvSharp.DistanceTypes.L2, 0, 0.01, 0.01);
    if (hasRight)
        rightLine  = OpenCvSharp.Cv2.FitLine(rightPts,  OpenCvSharp.DistanceTypes.L2, 0, 0.01, 0.01);

    int imgW = flines.Cols;
    int imgH = flines.Rows;
    float L  = (float)Math.Sqrt(imgW * imgW + imgH * imgH);

    Func<OpenCvSharp.Line2D, bool> isValid = line =>
    {
        float dx = line.P2.X - line.P1.X;
        float dy = line.P2.Y - line.P1.Y;
        return Math.Abs(dx) + Math.Abs(dy) > 1e-6f;
    };

    // draw (almost) infinite line across the image
    Action<OpenCvSharp.Line2D> drawLine = line =>
    {
        if (!isValid(line)) return;

        float dx  = line.P2.X - line.P1.X;
        float dy  = line.P2.Y - line.P1.Y;
        float len = (float)Math.Sqrt(dx * dx + dy * dy);
        if (len < 1e-6f) return;

        float vx = dx / len;
        float vy = dy / len;
        var p0   = line.P1;

        var p1 = new OpenCvSharp.Point(
            (int)Math.Round(p0.X - vx * L),
            (int)Math.Round(p0.Y - vy * L));
        var p2 = new OpenCvSharp.Point(
            (int)Math.Round(p0.X + vx * L),
            (int)Math.Round(p0.Y + vy * L));

        OpenCvSharp.Cv2.Line(flines, p1, p2, new OpenCvSharp.Scalar(0, 255, 0), 2);
    };

    if (hasBottom) drawLine(bottomLine);
    if (hasTop)    drawLine(topLine);
    if (hasLeft)   drawLine(leftLine);
    if (hasRight)  drawLine(rightLine);

    // intersection of two infinite Line2D's
    Func<OpenCvSharp.Line2D, OpenCvSharp.Line2D, OpenCvSharp.Point2f?> intersect =
        (l1, l2) =>
        {
            if (!isValid(l1) || !isValid(l2)) return null;

            var p1 = new OpenCvSharp.Point2f(l1.P1.X, l1.P1.Y);
            var d1 = new OpenCvSharp.Point2f(l1.P2.X - l1.P1.X, l1.P2.Y - l1.P1.Y);

            var p2 = new OpenCvSharp.Point2f(l2.P1.X, l2.P1.Y);
            var d2 = new OpenCvSharp.Point2f(l2.P2.X - l2.P1.X, l2.P2.Y - l2.P1.Y);

            float det = d1.X * d2.Y - d1.Y * d2.X;
            if (Math.Abs(det) < 1e-6f) return null; // parallel

            float t = ((p2.X - p1.X) * d2.Y - (p2.Y - p1.Y) * d2.X) / det;
            return new OpenCvSharp.Point2f(p1.X + t * d1.X, p1.Y + t * d1.Y);
        };

    var corners = new System.Collections.Generic.List<OpenCvSharp.Point>();

    var cBL = (hasBottom && hasLeft)  ? intersect(bottomLine, leftLine)  : null; // bottom-left
    var cBR = (hasBottom && hasRight) ? intersect(bottomLine, rightLine) : null; // bottom-right
    var cTL = (hasTop && hasLeft)     ? intersect(topLine,    leftLine)  : null; // top-left
    var cTR = (hasTop && hasRight)    ? intersect(topLine,    rightLine) : null; // top-right

    if (cBL.HasValue) corners.Add(new OpenCvSharp.Point((int)Math.Round(cBL.Value.X), (int)Math.Round(cBL.Value.Y)));
    if (cBR.HasValue) corners.Add(new OpenCvSharp.Point((int)Math.Round(cBR.Value.X), (int)Math.Round(cBR.Value.Y)));
    if (cTL.HasValue) corners.Add(new OpenCvSharp.Point((int)Math.Round(cTL.Value.X), (int)Math.Round(cTL.Value.Y)));
    if (cTR.HasValue) corners.Add(new OpenCvSharp.Point((int)Math.Round(cTR.Value.X), (int)Math.Round(cTR.Value.Y)));

    // Draw crosses at corners
    int crossHalf = 10;
    foreach (var c in corners)
    {
        OpenCvSharp.Cv2.Line(
            flines,
            new OpenCvSharp.Point(c.X - crossHalf, c.Y),
            new OpenCvSharp.Point(c.X + crossHalf, c.Y),
            new OpenCvSharp.Scalar(0, 0, 255),
            2);

        OpenCvSharp.Cv2.Line(
            flines,
            new OpenCvSharp.Point(c.X, c.Y - crossHalf),
            new OpenCvSharp.Point(c.X, c.Y + crossHalf),
            new OpenCvSharp.Scalar(0, 0, 255),
            2);
    }
}
// ---- end block ----