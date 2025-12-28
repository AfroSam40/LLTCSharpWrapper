using System;
using System.Drawing;
using OpenCvSharp;
using OpenCvSharp.Extensions;

public static class DogboneDetector
{
    /// <summary>
    /// Detect the large rectangular/square void surrounded by holes and
    /// the "dogbone" (mouse-ear) corner using OpenCV,
    /// then annotate the rectangle and the opposite corner.
    /// 
    /// Assumes: holes are WHITE in the input bitmap, background is dark.
    /// </summary>
    /// <param name="input">Input bitmap (projection from point cloud).</param>
    /// <param name="dogboneCorner">Output: corner with extra circular cut.</param>
    /// <param name="oppositeCorner">Output: corner diagonally opposite the dogbone.</param>
    /// <param name="holeThreshold">
    /// Intensity threshold used to binarize holes (0–255).
    /// Pixels ≥ threshold become "hole" (white) in the binary mask.
    /// </param>
    /// <param name="morphKernelSize">
    /// Size of the morphological opening kernel used to clean noise (set 0 to skip).
    /// </param>
    /// <param name="minVoidAreaFraction">
    /// Minimum fraction of image area a void must have to be considered (e.g. 0.05 = 5%).
    /// </param>
    /// <param name="cornerPatchRadius">
    /// Half-size (in pixels) of the square patch around each corner used
    /// to measure "hole density" for dogbone detection.
    /// </param>
    /// <returns>Annotated bitmap. If detection fails, a copy of the input is returned.</returns>
    public static Bitmap AnnotateBigRectVoidWithDogboneCv(
        Bitmap input,
        out Point2f dogboneCorner,
        out Point2f oppositeCorner,
        double holeThreshold = 64,
        int morphKernelSize = 3,
        double minVoidAreaFraction = 0.05,
        int cornerPatchRadius = 20)
    {
        dogboneCorner = new Point2f();
        oppositeCorner = new Point2f();

        if (input == null)
            throw new ArgumentNullException(nameof(input));

        // --- 0. Convert Bitmap -> Mat ---
        Mat src = BitmapConverter.ToMat(input);

        // Ensure we have something we can draw color on later
        Mat color;
        if (src.Channels() == 1)
        {
            Cv2.CvtColor(src, color, ColorConversionCodes.GRAY2BGR);
        }
        else if (src.Channels() == 3)
        {
            color = src.Clone();
        }
        else if (src.Channels() == 4)
        {
            Cv2.CvtColor(src, color, ColorConversionCodes.BGRA2BGR);
        }
        else
        {
            throw new InvalidOperationException("Unsupported channel count in input image.");
        }

        // --- 1. Make a binary mask where HOLES are white ---
        Mat gray = new Mat();
        if (src.Channels() == 3)
            Cv2.CvtColor(src, gray, ColorConversionCodes.BGR2GRAY);
        else if (src.Channels() == 4)
            Cv2.CvtColor(src, gray, ColorConversionCodes.BGRA2GRAY);
        else
            gray = src.Clone();

        Mat bin = new Mat();
        Cv2.Threshold(gray, bin, holeThreshold, 255, ThresholdTypes.Binary);

        // Optional: small opening to clean noise
        if (morphKernelSize > 0)
        {
            Mat kernel = Cv2.GetStructuringElement(
                MorphShapes.Rect,
                new OpenCvSharp.Size(morphKernelSize, morphKernelSize));
            Cv2.MorphologyEx(bin, bin, MorphTypes.Open, kernel);
        }

        // --- 2. Invert: central VOID becomes white ---
        Mat inv = new Mat();
        Cv2.BitwiseNot(bin, inv);

        // --- 3. Find external contours on inverted image ---
        Cv2.FindContours(
            inv,
            out OpenCvSharp.Point[][] contours,
            out HierarchyIndex[] hierarchy,
            RetrievalModes.External,
            ContourApproximationModes.ApproxSimple);

        if (contours == null || contours.Length == 0)
            return BitmapConverter.ToBitmap(color); // nothing found

        double imgArea = inv.Rows * inv.Cols;
        double minVoidArea = imgArea * minVoidAreaFraction;

        RotatedRect? bestRect = null;
        double bestScore = double.NegativeInfinity;

        // Choose the large, central-ish rectangular void
        foreach (var contour in contours)
        {
            if (contour.Length < 4)
                continue;

            RotatedRect rect = Cv2.MinAreaRect(contour);
            double area = rect.Size.Width * rect.Size.Height;
            if (area < minVoidArea)
                continue;

            // Penalize distance from image center
            var c = rect.Center;
            double dx = c.X - inv.Cols / 2.0;
            double dy = c.Y - inv.Rows / 2.0;
            double dist2 = dx * dx + dy * dy;

            double score = area / (1.0 + 0.001 * dist2);
            if (score > bestScore)
            {
                bestScore = score;
                bestRect = rect;
            }
        }

        if (bestRect == null)
            return BitmapConverter.ToBitmap(color); // no suitable void

        RotatedRect voidRect = bestRect.Value;
        Point2f[] rectCorners = voidRect.Points(); // 4 corners in some order

        // --- 4. Find DOGBONE corner by local hole density around each corner ---
        double bestHoleFrac = double.NegativeInfinity;
        Point2f bestDogbone = rectCorners[0];

        foreach (var c in rectCorners)
        {
            int cx = (int)Math.Round(c.X);
            int cy = (int)Math.Round(c.Y);

            int x0 = Math.Max(0, cx - cornerPatchRadius);
            int y0 = Math.Max(0, cy - cornerPatchRadius);
            int x1 = Math.Min(bin.Cols - 1, cx + cornerPatchRadius);
            int y1 = Math.Min(bin.Rows - 1, cy + cornerPatchRadius);

            if (x1 <= x0 || y1 <= y0)
                continue;

            var roi = new Mat(bin, new Rect(x0, y0, x1 - x0 + 1, y1 - y0 + 1));
            double white = Cv2.CountNonZero(roi);
            double frac = white / roi.Total();   // fraction of hole pixels

            if (frac > bestHoleFrac)
            {
                bestHoleFrac = frac;
                bestDogbone = c;
            }
        }

        dogboneCorner = bestDogbone;

        // --- 5. Opposite corner: farthest one from dogbone ---
        Point2f bestOpp = rectCorners[0];
        double maxD2 = double.NegativeInfinity;

        foreach (var c in rectCorners)
        {
            double dx = c.X - dogboneCorner.X;
            double dy = c.Y - dogboneCorner.Y;
            double d2 = dx * dx + dy * dy;
            if (d2 > maxD2)
            {
                maxD2 = d2;
                bestOpp = c;
            }
        }

        oppositeCorner = bestOpp;

        // --- 6. Draw annotation on 'color' Mat ---
        // Draw rectangle
        for (int i = 0; i < 4; i++)
        {
            Point2f p0 = rectCorners[i];
            Point2f p1 = rectCorners[(i + 1) % 4];
            Cv2.Line(color, (Point)p0, (Point)p1, new Scalar(0, 255, 0), 2);  // green
        }

        // Dogbone corner (red)
        Cv2.Circle(color, (Point)dogboneCorner, 6, new Scalar(0, 0, 255), -1);

        // Opposite corner (blue)
        Cv2.Circle(color, (Point)oppositeCorner, 6, new Scalar(255, 0, 0), -1);

        // --- 7. Back to Bitmap ---
        return BitmapConverter.ToBitmap(color);
    }
}