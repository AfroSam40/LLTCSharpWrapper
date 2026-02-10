using System;
using System.Collections.Generic;
using System.Linq;
using System.Windows.Media.Media3D;
using OpenCvSharp;

public static class VoidDetection
{
    /// <summary>
    /// Finds centers of "voids" (holes) on/near a surface plane.
    /// Approach:
    ///   1) Project points to the best-fit plane (PlaneFitResult).
    ///   2) Rasterize projected points into a binary occupancy image (solid = 1).
    ///   3) Invert to get "void" mask (void = 1).
    ///   4) ConnectedComponents on void mask.
    ///   5) Keep only voids whose equivalent radius >= minVoidRadius.
    ///   6) Convert component centroids back to 3D points on the plane.
    ///
    /// Notes:
    /// - You must pass the point cloud for the surface region you are analyzing (ideally already cropped to the plate area).
    /// - pixelSize is in your point units (mm if your point cloud is mm).
    /// - Returns centers on the plane (Z determined by plane at XY).
    /// </summary>
    public static List<Point3D> FindVoidCentersOnPlane(
        PlaneFitResult plane,
        Point3DCollection points,
        double pixelSize,
        double minVoidRadius,
        double padding = 2.0,
        int closeKernel = 0 // 0 = no closing, otherwise typical 3..11
    )
    {
        if (plane == null) throw new ArgumentNullException(nameof(plane));
        if (points == null || points.Count == 0) return new List<Point3D>();
        if (pixelSize <= 0) throw new ArgumentOutOfRangeException(nameof(pixelSize));
        if (minVoidRadius <= 0) throw new ArgumentOutOfRangeException(nameof(minVoidRadius));

        // --- Build a stable plane basis (U,V) from the plane normal ---
        var n = plane.Normal;
        if (n.LengthSquared < 1e-12)
        {
            // Fallback normal for your z=a*x+b*y+c plane form: (A, B, -1)
            n = new Vector3D(plane.A, plane.B, -1.0);
        }
        if (n.LengthSquared < 1e-12) n = new Vector3D(0, 0, 1);
        n.Normalize();

        // Pick a non-parallel reference axis to build U
        Vector3D refAxis = Math.Abs(Vector3D.DotProduct(n, new Vector3D(0, 0, 1))) < 0.9
            ? new Vector3D(0, 0, 1)
            : new Vector3D(1, 0, 0);

        Vector3D u = Vector3D.CrossProduct(refAxis, n);
        if (u.LengthSquared < 1e-12) u = Vector3D.CrossProduct(new Vector3D(0, 1, 0), n);
        u.Normalize();

        Vector3D v = Vector3D.CrossProduct(n, u);
        v.Normalize();

        // Choose plane origin (centroid is fine if you are working local)
        Point3D origin = plane.Centroid;

        // --- Project all points onto plane coordinates (s,t) ---
        // s = dot((p-origin), u), t = dot((p-origin), v)
        var st = new (double s, double t)[points.Count];
        double minS = double.PositiveInfinity, maxS = double.NegativeInfinity;
        double minT = double.PositiveInfinity, maxT = double.NegativeInfinity;

        for (int i = 0; i < points.Count; i++)
        {
            Vector3D w = points[i] - origin;
            double s = Vector3D.DotProduct(w, u);
            double t = Vector3D.DotProduct(w, v);
            st[i] = (s, t);

            if (s < minS) minS = s;
            if (s > maxS) maxS = s;
            if (t < minT) minT = t;
            if (t > maxT) maxT = t;
        }

        // Expand bounds slightly so edge voids don't get clipped
        double pad = Math.Max(0, padding);
        minS -= pad; maxS += pad;
        minT -= pad; maxT += pad;

        int width = (int)Math.Ceiling((maxS - minS) / pixelSize) + 1;
        int height = (int)Math.Ceiling((maxT - minT) / pixelSize) + 1;

        // Guard against huge images
        if (width < 10 || height < 10) return new List<Point3D>();
        if ((long)width * height > 200_000_000) // ~200 MP guard
            throw new InvalidOperationException($"Raster too large: {width}x{height}. Increase pixelSize or crop points.");

        // --- Rasterize occupied pixels ---
        // occupied = 255 (white), background = 0
        Mat occ = new Mat(height, width, MatType.CV_8UC1, Scalar.All(0));

        for (int i = 0; i < st.Length; i++)
        {
            int x = (int)Math.Round((st[i].s - minS) / pixelSize);
            int y = (int)Math.Round((st[i].t - minT) / pixelSize);

            if ((uint)x < (uint)width && (uint)y < (uint)height)
                occ.Set(y, x, 255);
        }

        // Optional: close gaps automatically (fills sparse sampling holes between points)
        if (closeKernel > 0)
        {
            int k = closeKernel % 2 == 0 ? closeKernel + 1 : closeKernel; // force odd
            k = Math.Max(3, k);
            using var kernel = Cv2.GetStructuringElement(MorphShapes.Ellipse, new Size(k, k));
            Cv2.MorphologyEx(occ, occ, MorphTypes.Close, kernel);
        }

        // --- Invert occupancy to get voids ---
        // voids = 255 where there is NO material
        Mat voids = new Mat();
        Cv2.BitwiseNot(occ, voids);

        // --- Connected components on voids ---
        // We want connected WHITE regions (255). Ensure it's binary.
        Cv2.Threshold(voids, voids, 127, 255, ThresholdTypes.Binary);

        // Connected components returns labels for each region
        Mat labels = new Mat();
        Mat stats = new Mat();
        Mat centroids = new Mat();
        int nLabels = Cv2.ConnectedComponentsWithStats(voids, labels, stats, centroids, PixelConnectivity.Connectivity8);

        // Minimum area corresponding to minVoidRadius
        // area_px = (pi * r^2) / (pixelSize^2)
        double minAreaPx = Math.PI * minVoidRadius * minVoidRadius / (pixelSize * pixelSize);

        var results = new List<Point3D>();

        // label 0 is background; void regions start at 1
        for (int lbl = 1; lbl < nLabels; lbl++)
        {
            int area = stats.At<int>(lbl, (int)ConnectedComponentsTypes.Area);
            if (area <= 0) continue;

            // Equivalent radius (same-area circle) in world units
            double eqRadius = Math.Sqrt(area / Math.PI) * pixelSize;
            if (eqRadius < minVoidRadius) continue;

            double cx = centroids.At<double>(lbl, 0);
            double cy = centroids.At<double>(lbl, 1);

            // Convert pixel centroid back to (s,t)
            double sC = minS + cx * pixelSize;
            double tC = minT + cy * pixelSize;

            // Map back to 3D point on plane: P = origin + s*u + t*v
            Point3D center3D = origin + (sC * u) + (tC * v);

            results.Add(center3D);
        }

        // If you want to exclude the "outside-of-part" void (the big background),
        // you can remove the largest-area component, which is often the exterior region:
        // (Uncomment if needed.)
        /*
        if (results.Count > 1)
        {
            // Find largest by area and remove it
            int bestLbl = -1;
            int bestArea = -1;
            for (int lbl = 1; lbl < nLabels; lbl++)
            {
                int area = stats.At<int>(lbl, (int)ConnectedComponentsTypes.Area);
                if (area > bestArea) { bestArea = area; bestLbl = lbl; }
            }
            if (bestLbl > 0)
            {
                // Remove corresponding centroid from results (recompute list matching)
                // Simpler: just remove the point closest to that centroid
                double cx = centroids.At<double>(bestLbl, 0);
                double cy = centroids.At<double>(bestLbl, 1);
                double sC = minS + cx * pixelSize;
                double tC = minT + cy * pixelSize;
                Point3D bg = origin + (sC * u) + (tC * v);

                int idx = -1;
                double bestD2 = double.MaxValue;
                for (int i = 0; i < results.Count; i++)
                {
                    double d2 = (results[i] - bg).LengthSquared;
                    if (d2 < bestD2) { bestD2 = d2; idx = i; }
                }
                if (idx >= 0) results.RemoveAt(idx);
            }
        }
        */

        return results;
    }
}