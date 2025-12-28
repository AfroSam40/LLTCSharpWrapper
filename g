using OpenCvSharp;
using System.Drawing;
using Bitmap = System.Drawing.Bitmap;

public static class RectAnnotator
{
    private static void DrawCross(Mat img, Point center, int size, Scalar color, int thickness = 2)
    {
        Cv2.Line(img,
            new OpenCvSharp.Point(center.X - size, center.Y),
            new OpenCvSharp.Point(center.X + size, center.Y),
            color, thickness);

        Cv2.Line(img,
            new OpenCvSharp.Point(center.X, center.Y - size),
            new OpenCvSharp.Point(center.X, center.Y + size),
            color, thickness);
    }

    /// <summary>
    /// Starting from a (possibly grayscale) bitmap where the voids are white after inversion,
    /// run Connected Components, find rectangular/square blobs, and annotate each corner
    /// with a cross. Returns an annotated bitmap.
    /// </summary>
    public static Bitmap AnnotateRectanglesWithCC(
        Bitmap input,
        int morphKernelSize = 3,
        int minArea = 200,
        double maxAspectRatio = 4.0)
    {
        if (input == null) throw new ArgumentNullException(nameof(input));

        // --- 1. Convert to Mat ---
        using var src = BitmapConverter.ToMat(input);

        // Ensure we have a single-channel grayscale for thresholding
        using var gray = new Mat();
        if (src.Channels() == 1)
        {
            src.CopyTo(gray);
        }
        else
        {
            Cv2.CvtColor(src, gray, ColorConversionCodes.BGR2GRAY);
        }

        // --- 2. Threshold (holes => white, background => black) ---
        using var bin = new Mat();
        // Otsu tends to work well; you can change to a fixed threshold if needed
        Cv2.Threshold(gray, bin, 0, 255, ThresholdTypes.Binary | ThresholdTypes.Otsu);

        // --- 3. Optional morphology to close gaps between points ---
        if (morphKernelSize > 0)
        {
            using var kernel = Cv2.GetStructuringElement(
                MorphShapes.Rect,
                new OpenCvSharp.Size(morphKernelSize, morphKernelSize));
            Cv2.MorphologyEx(bin, bin, MorphTypes.Close, kernel);
        }

        // --- 4. Invert so "void region" is white (255), background is black (0) ---
        using var inv = new Mat();
        Cv2.BitwiseNot(bin, inv);

        // --- 5. Connected Components on the inverted image ---
        using var labels = new Mat();
        using var stats = new Mat();
        using var centroids = new Mat();
        int nLabels = Cv2.ConnectedComponentsWithStats(
            inv,
            labels,
            stats,
            centroids,
            PixelConnectivity.Connectivity8,
            MatType.CV_32S);

        // --- 6. Prepare color image to draw on ---
        using var color = new Mat();
        if (src.Channels() == 1)
            Cv2.CvtColor(src, color, ColorConversionCodes.GRAY2BGR);
        else
            src.CopyTo(color);

        // --- 7. Loop over components (label 0 = background, skip it) ---
        for (int label = 1; label < nLabels; label++)
        {
            int area = stats.Get<int>(label, (int)ConnectedComponentsTypes.Area);
            if (area < minArea)
                continue; // too small, probably noise

            int left   = stats.Get<int>(label, (int)ConnectedComponentsTypes.Left);
            int top    = stats.Get<int>(label, (int)ConnectedComponentsTypes.Top);
            int width  = stats.Get<int>(label, (int)ConnectedComponentsTypes.Width);
            int height = stats.Get<int>(label, (int)ConnectedComponentsTypes.Height);

            if (width <= 0 || height <= 0)
                continue;

            double aspect = (double)Math.Max(width, height) / Math.Min(width, height);

            // Rough "rectangular-ish / squareish" filter:
            //   aspect close-ish to 1 (square) or not insanely elongated (rectangles).
            if (aspect > maxAspectRatio)
                continue;

            // --- 8. Rectangle corners in image coordinates ---
            var c0 = new Point(left, top);
            var c1 = new Point(left + width - 1, top);
            var c2 = new Point(left + width - 1, top + height - 1);
            var c3 = new Point(left, top + height - 1);

            // Draw rectangle outline (optional)
            Cv2.Rectangle(color, new OpenCvSharp.Rect(left, top, width, height),
                new Scalar(0, 255, 0), 2);

            // Draw crosses at corners
            DrawCross(color, c0, 6, new Scalar(0, 0, 255), 2);
            DrawCross(color, c1, 6, new Scalar(0, 0, 255), 2);
            DrawCross(color, c2, 6, new Scalar(0, 0, 255), 2);
            DrawCross(color, c3, 6, new Scalar(0, 0, 255), 2);
        }

        // --- 9. Convert back to Bitmap ---
        return BitmapConverter.ToBitmap(color);
    }
}