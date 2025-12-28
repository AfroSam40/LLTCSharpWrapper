// ---- Fit lines to the four sides of `contour` and draw lines + corner crosses ----
if (contour != null && contour.Length >= 4)
{
    // Convert contour to float points
    var pts = contour.Select(p => new OpenCvSharp.Point2f(p.X, p.Y)).ToList();

    // Bounding box of contour
    float minX = pts.Min(p => p.X);
    float maxX = pts.Max(p => p.X);
    float minY = pts.Min(p => p.Y);
    float maxY = pts.Max(p => p.Y);

    // Band thickness (how far from each edge we still consider "on that side")
    float band = 0.05f * Math.Max(maxX - minX, maxY - minY); // 5% of size
    int   minSidePts = 10;

    var bottomPts = new List<OpenCvSharp.Point2f>();
    var topPts    = new List<OpenCvSharp.Point2f>();
    var leftPts   = new List<OpenCvSharp.Point2f>();
    var rightPts  = new List<OpenCvSharp.Point2f>();

    // Classify contour points to four sides based on proximity to bbox edges
    foreach (var p in pts)
    {
        if (Math.Abs(p.Y - maxY) <= band) bottomPts.Add(p); // bottom edge
        if (Math.Abs(p.Y - minY) <= band) topPts.Add(p);    // top edge
        if (Math.Abs(p.X - minX) <= band) leftPts.Add(p);   // left edge
        if (Math.Abs(p.X - maxX) <= band) rightPts.Add(p);  // right edge
    }

    // Fit lines: each Vec4f is (vx, vy, x0, y0)
    OpenCvSharp.Vec4f lineBottom = new OpenCvSharp.Vec4f(0, 0, 0, 0);
    OpenCvSharp.Vec4f lineTop    = new OpenCvSharp.Vec4f(0, 0, 0, 0);
    OpenCvSharp.Vec4f lineLeft   = new OpenCvSharp.Vec4f(0, 0, 0, 0);
    OpenCvSharp.Vec4f lineRight  = new OpenCvSharp.Vec4f(0, 0, 0, 0);

    bool hasBottom = bottomPts.Count >= minSidePts;
    bool hasTop    = topPts.Count    >= minSidePts;
    bool hasLeft   = leftPts.Count   >= minSidePts;
    bool hasRight  = rightPts.Count  >= minSidePts;

    if (hasBottom)
        lineBottom = OpenCvSharp.Cv2.FitLine(
            bottomPts, OpenCvSharp.DistanceTypes.L2, 0, 0.01, 0.01);
    if (hasTop)
        lineTop = OpenCvSharp.Cv2.FitLine(
            topPts, OpenCvSharp.DistanceTypes.L2, 0, 0.01, 0.01);
    if (hasLeft)
        lineLeft = OpenCvSharp.Cv2.FitLine(
            leftPts, OpenCvSharp.DistanceTypes.L2, 0, 0.01, 0.01);
    if (hasRight)
        lineRight = OpenCvSharp.Cv2.FitLine(
            rightPts, OpenCvSharp.DistanceTypes.L2, 0, 0.01, 0.01);

    int imgW = flines.Cols;
    int imgH = flines.Rows;
    float L = (float)Math.Sqrt(imgW * imgW + imgH * imgH);

    // Draw a line helper (inline, no extra methods)
    Action<OpenCvSharp.Vec4f> drawLine = line =>
    {
        if (line.Item0 == 0 && line.Item1 == 0 && line.Item2 == 0 && line.Item3 == 0)
            return;

        float vx = line.Item0;
        float vy = line.Item1;
        float x0 = line.Item2;
        float y0 = line.Item3;

        var p1 = new OpenCvSharp.Point(
            (int)Math.Round(x0 - vx * L),
            (int)Math.Round(y0 - vy * L));
        var p2 = new OpenCvSharp.Point(
            (int)Math.Round(x0 + vx * L),
            (int)Math.Round(y0 + vy * L));

        OpenCvSharp.Cv2.Line(
            flines,
            p1,
            p2,
            new OpenCvSharp.Scalar(0, 255, 0), // green
            2);
    };

    // Draw the four side lines
    if (hasBottom) drawLine(lineBottom);
    if (hasTop)    drawLine(lineTop);
    if (hasLeft)   drawLine(lineLeft);
    if (hasRight)  drawLine(lineRight);

    // Compute intersections (corners)
    var corners = new List<OpenCvSharp.Point>();

    // inline intersection for pairs
    Action<OpenCvSharp.Vec4f, OpenCvSharp.Vec4f> addIntersection =
        (l1, l2) =>
        {
            if ((l1.Item0 == 0 && l1.Item1 == 0 && l1.Item2 == 0 && l1.Item3 == 0) ||
                (l2.Item0 == 0 && l2.Item1 == 0 && l2.Item2 == 0 && l2.Item3 == 0))
                return;

            float vx1 = l1.Item0, vy1 = l1.Item1, x01 = l1.Item2, y01 = l1.Item3;
            float vx2 = l2.Item0, vy2 = l2.Item1, x02 = l2.Item2, y02 = l2.Item3;

            double a1 = vy1, b1 = -vx1, c1 = -vy1 * x01 + vx1 * y01;
            double a2 = vy2, b2 = -vx2, c2 = -vy2 * x02 + vx2 * y02;

            double D = a1 * b2 - a2 * b1;
            if (Math.Abs(D) < 1e-6) return; // almost parallel

            double x = (b1 * c2 - b2 * c1) / D;
            double y = (c1 * a2 - c2 * a1) / D;

            corners.Add(new OpenCvSharp.Point(
                (int)Math.Round(x),
                (int)Math.Round(y)));
        };

    if (hasBottom && hasLeft)  addIntersection(lineBottom, lineLeft);   // bottom-left
    if (hasBottom && hasRight) addIntersection(lineBottom, lineRight);  // bottom-right
    if (hasTop && hasLeft)     addIntersection(lineTop, lineLeft);      // top-left
    if (hasTop && hasRight)    addIntersection(lineTop, lineRight);     // top-right

    // Draw crosses at each corner
    int crossHalf = 10;
    foreach (var c in corners)
    {
        // horizontal
        OpenCvSharp.Cv2.Line(
            flines,
            new OpenCvSharp.Point(c.X - crossHalf, c.Y),
            new OpenCvSharp.Point(c.X + crossHalf, c.Y),
            new OpenCvSharp.Scalar(0, 0, 255), 2); // red

        // vertical
        OpenCvSharp.Cv2.Line(
            flines,
            new OpenCvSharp.Point(c.X, c.Y - crossHalf),
            new OpenCvSharp.Point(c.X, c.Y + crossHalf),
            new OpenCvSharp.Scalar(0, 0, 255), 2);
    }
}
// ---- end block ----