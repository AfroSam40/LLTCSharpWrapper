// ---- Fit lines to the 4 sides of `contour`, draw them, and mark corners ----
if (contour != null && contour.Length >= 4)
{
    // Convert contour to float points
    var pts = contour.Select(p => new OpenCvSharp.Point2f(p.X, p.Y)).ToList();

    // Bounding box in image coordinates
    float minX = pts.Min(p => p.X);
    float maxX = pts.Max(p => p.X);
    float minY = pts.Min(p => p.Y);
    float maxY = pts.Max(p => p.Y);

    float size   = Math.Max(maxX - minX, maxY - minY);
    float band   = 0.05f * size;   // side “thickness” band = 5% of bbox size
    int   minPts = 10;             // minimum points to accept a side

    var bottomPts = new List<OpenCvSharp.Point2f>();
    var topPts    = new List<OpenCvSharp.Point2f>();
    var leftPts   = new List<OpenCvSharp.Point2f>();
    var rightPts  = new List<OpenCvSharp.Point2f>();

    // Classify points to 4 sides by closeness to bbox edges
    foreach (var p in pts)
    {
        if (Math.Abs(p.Y - maxY) <= band) bottomPts.Add(p); // bottom edge
        if (Math.Abs(p.Y - minY) <= band) topPts.Add(p);    // top edge
        if (Math.Abs(p.X - minX) <= band) leftPts.Add(p);   // left edge
        if (Math.Abs(p.X - maxX) <= band) rightPts.Add(p);  // right edge
    }

    bool hasBottom = bottomPts.Count >= minPts;
    bool hasTop    = topPts.Count    >= minPts;
    bool hasLeft   = leftPts.Count   >= minPts;
    bool hasRight  = rightPts.Count  >= minPts;

    OpenCvSharp.Vec4f bottomLine = new OpenCvSharp.Vec4f();
    OpenCvSharp.Vec4f topLine    = new OpenCvSharp.Vec4f();
    OpenCvSharp.Vec4f leftLine   = new OpenCvSharp.Vec4f();
    OpenCvSharp.Vec4f rightLine  = new OpenCvSharp.Vec4f();

    // Fit lines: Vec4f = (vx, vy, x0, y0)
    if (hasBottom)
        bottomLine = OpenCvSharp.Cv2.FitLine(
            bottomPts, OpenCvSharp.DistanceTypes.L2, 0, 0.01, 0.01);

    if (hasTop)
        topLine = OpenCvSharp.Cv2.FitLine(
            topPts, OpenCvSharp.DistanceTypes.L2, 0, 0.01, 0.01);

    if (hasLeft)
        leftLine = OpenCvSharp.Cv2.FitLine(
            leftPts, OpenCvSharp.DistanceTypes.L2, 0, 0.01, 0.01);

    if (hasRight)
        rightLine = OpenCvSharp.Cv2.FitLine(
            rightPts, OpenCvSharp.DistanceTypes.L2, 0, 0.01, 0.01);

    int imgW = flines.Cols;
    int imgH = flines.Rows;
    float L  = (float)Math.Sqrt(imgW * imgW + imgH * imgH); // “infinite” length

    // Helper to draw a fitted Vec4f line
    Action<OpenCvSharp.Vec4f> drawLine = line =>
    {
        float vx = line.Item0;
        float vy = line.Item1;
        float x0 = line.Item2;
        float y0 = line.Item3;

        float len = (float)Math.Sqrt(vx * vx + vy * vy);
        if (len < 1e-6f) return;

        vx /= len;
        vy /= len;

        var p1 = new OpenCvSharp.Point(
            (int)Math.Round(x0 - vx * L),
            (int)Math.Round(y0 - vy * L));
        var p2 = new OpenCvSharp.Point(
            (int)Math.Round(x0 + vx * L),
            (int)Math.Round(y0 + vy * L));

        OpenCvSharp.Cv2.Line(flines, p1, p2, new OpenCvSharp.Scalar(0, 255, 0), 2);
    };

    if (hasBottom) drawLine(bottomLine);
    if (hasTop)    drawLine(topLine);
    if (hasLeft)   drawLine(leftLine);
    if (hasRight)  drawLine(rightLine);

    // Compute intersections (corners) of pairs of Vec4f lines
    var corners = new System.Collections.Generic.List<OpenCvSharp.Point>();

    Action<OpenCvSharp.Vec4f, OpenCvSharp.Vec4f> addIntersection =
        (l1, l2) =>
        {
            float vx1 = l1.Item0, vy1 = l1.Item1, x01 = l1.Item2, y01 = l1.Item3;
            float vx2 = l2.Item0, vy2 = l2.Item1, x02 = l2.Item2, y02 = l2.Item3;

            float denom = vx1 * vy2 - vy1 * vx2;
            if (Math.Abs(denom) < 1e-6f) return; // parallel

            float t1 = ((x02 - x01) * vy2 - (y02 - y01) * vx2) / denom;
            float xi = x01 + vx1 * t1;
            float yi = y01 + vy1 * t1;

            corners.Add(new OpenCvSharp.Point(
                (int)Math.Round(xi),
                (int)Math.Round(yi)));
        };

    if (hasBottom && hasLeft)  addIntersection(bottomLine, leftLine);   // BL
    if (hasBottom && hasRight) addIntersection(bottomLine, rightLine);  // BR
    if (hasTop    && hasLeft)  addIntersection(topLine,    leftLine);   // TL
    if (hasTop    && hasRight) addIntersection(topLine,    rightLine);  // TR

    // Draw red crosses at each corner
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