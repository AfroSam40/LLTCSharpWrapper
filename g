using System;
using System.Collections.Generic;
using System.Linq;
using System.Windows;                 // For System.Windows.Point
using OpenCvSharp;                    // For Mat, Point2f, Cv2

public static class HullHelpers
{
    /// <summary>
    /// Computes a 2D convex hull using OpenCvSharp's ConvexHull.
    /// Input/Output are in System.Windows.Point.
    /// </summary>
    public static List<Point> ComputeConvexHull(IList<Point> points)
    {
        var hullPoints = new List<Point>();

        if (points == null || points.Count == 0)
            return hullPoints;

        // --- Convert WPF Points -> OpenCvSharp Point2f ---
        var pts2f = points.Select(p => new Point2f((float)p.X, (float)p.Y)).ToArray();

        // Put into a Mat as a Nx1 CV_32FC2 (like a contour)
        using (var src = new Mat(pts2f.Length, 1, MatType.CV_32FC2))
        {
            src.SetArray(0, 0, pts2f);

            // Compute convex hull; this overload returns the hull as another Mat
            // with the same type / layout as src (Nx1 CV_32FC2).
            using (var hullMat = src.ConvexHull(clockwise: false, returnPoints: true))
            {
                // Read back as Point2f[]
                var hull2f = hullMat.GetArray<Point2f>();

                // Convert back to WPF Points
                foreach (var p in hull2f)
                    hullPoints.Add(new Point(p.X, p.Y));
            }
        }

        return hullPoints;
    }
}