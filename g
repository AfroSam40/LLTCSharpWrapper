using System;
using System.Drawing;
using OpenCvSharp;
using OpenCvSharp.Extensions;

public static class DogboneHoleDetectorCv
{
    /// <summary>
    /// Find a rectangular/square hole with a dogbone "mouse ear" on one corner
    /// using OpenCV, and annotate the opposite corner.
    /// </summary>
    public static (Mat annotated, Point2f? oppositeCorner) AnnotateDogboneOppositeCornerCv(
        Bitmap input,
        double holeThreshold = 64,
        double earCornerRatioThreshold = 1.2)
    {
        if (input == null) throw new ArgumentNullException(nameof(input));

        // Convert Bitmap -> Mat
        Mat src = BitmapConverter.ToMat(input);

        // 1) Grayscale + threshold (hole = white)
        Mat gray = new Mat();
        Cv2.CvtColor(src, gray, ColorConversionCodes.BGR2GRAY);

        Mat bin = new Mat();
        Cv2.Threshold(gray, bin, holeThreshold, 255, ThresholdTypes.BinaryInv);

        // Optional: small open to remove noise
        Mat kernel = Cv2.GetStructuringElement(MorphShapes.Rect, new Size(3, 3));
        Cv2.MorphologyEx(bin, bin, MorphTypes.Open, kernel);

        // 2) Find external contours (each hole)
        Cv2.FindContours(bin, out Point[][] contours, out HierarchyIndex[] hier,
            RetrievalModes.External, ContourApproximationModes.Simple);

        float bestScore = 0;
        Point2f? bestOppCorner = null;
        int bestIdx = -1;

        foreach (var contour in contours)
        {
            double area = Cv2.ContourArea(contour);
            if (area < 50) continue; // ignore tiny noise

            var rect = Cv2.BoundingRect(contour);
            if (rect.Width < 5 || rect.Height < 5) continue;

            double aspect = (double)rect.Width / rect.Height;
            if (aspect < 0.5 || aspect > 2.0) continue; // roughly rectangular

            // centroid
            var m = Cv2.Moments(contour);
            if (Math.Abs(m.M00) < 1e-6) continue;
            float cx = (float)(m.M10 / m.M00);
            float cy = (float)(m.M01 / m.M00);

            // corners of bounding box
            var corners = new[]
            {
                new Point2f(rect.Left,            rect.Top),
                new Point2f(rect.Right,           rect.Top),
                new Point2f(rect.Right,           rect.Bottom),
                new Point2f(rect.Left,            rect.Bottom)
            };

            double[] d2 = new double[4];
            for (int i = 0; i < 4; i++)
            {
                double dx = corners[i].X - cx;
                double dy = corners[i].Y - cy;
                d2[i] = dx * dx + dy * dy;
            }

            double maxD2 = d2[0], minD2 = d2[0];
            int maxIdx = 0;
            for (int i = 1; i < 4; i++)
            {
                if (d2[i] > maxD2) { maxD2 = d2[i]; maxIdx = i; }
                if (d2[i] < minD2) minD2 = d2[i];
            }

            double ratio = maxD2 / Math.Max(minD2, 1e-6);
            if (ratio < earCornerRatioThreshold)
                continue; // corners all similar distance → plain rectangle, no ear

            if (ratio > bestScore)
            {
                bestScore = (float)ratio;
                bestIdx = Array.IndexOf(contours, contour);

                // farthest bounding-box corner is opposite to the ear
                bestOppCorner = corners[maxIdx];
            }
        }

        Mat annotated = src.Clone();

        if (bestOppCorner.HasValue)
        {
            var p = bestOppCorner.Value;
            // draw red crosshair
            Cv2.DrawMarker(
                annotated,
                (Point)p,
                new Scalar(0, 0, 255),
                MarkerTypes.Cross,
                16,
                2);
            Cv2.Circle(
                annotated,
                (Point)p,
                8,
                new Scalar(0, 0, 255),
                2);
        }

        return (annotated, bestOppCorner);
    }
}