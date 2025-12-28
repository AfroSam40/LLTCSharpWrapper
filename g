using OpenCvSharp;

private static Vec4f FitLineFromPoints(List<Point2f> pts)
{
    // Guard
    if (pts == null || pts.Count < 2)
        return new Vec4f(0, 0, 0, 0);

    // Build an Nx1 CV_32FC2 matrix from the points
    using var ptsMat = new Mat(pts.Count, 1, MatType.CV_32FC2);
    for (int i = 0; i < pts.Count; i++)
    {
        // Mat.Set(row, col, Point2f) is supported for CV_32FC2
        ptsMat.Set(i, 0, pts[i]);
    }

    // Destination Mat for the line (vx, vy, x0, y0)
    using var lineMat = new Mat(4, 1, MatType.CV_32FC1);

    // NOTE: no "out" here – second arg is OutputArray, not out Vec4f
    Cv2.FitLine(
        ptsMat,
        lineMat,
        DistanceTypes.L2,
        0,
        0.01,
        0.01);

    // Extract (vx, vy, x0, y0) from the 4x1 float Mat
    float vx = lineMat.Get<float>(0);
    float vy = lineMat.Get<float>(1);
    float x0 = lineMat.Get<float>(2);
    float y0 = lineMat.Get<float>(3);

    return new Vec4f(vx, vy, x0, y0);
}