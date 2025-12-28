// --- helpers inside RectSideFitter ---

private static Vec4f FitLineFromPoints(List<Point2f> pts)
{
    // Not enough points -> "invalid" line
    if (pts == null || pts.Count < 2)
        return new Vec4f(0, 0, 0, 0);

    // This overload takes IEnumerable<Point2f> directly
    return Cv2.FitLine(
        pts,                // IEnumerable<Point2f>
        DistanceTypes.L2,
        0,                  // param (ignored for L2)
        0.01,               // reps
        0.01                // aeps
    );
}