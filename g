using System;
using System.Collections.Generic;
using System.Linq;
using System.Windows.Media.Media3D;

namespace PointCloudUtils
{
    /// <summary>
    /// Result of fitting a (roughly horizontal) plane z = a*x + b*y + c.
    /// </summary>
    public class PlaneFitResult
    {
        /// <summary>
        /// Plane coefficients: z = A * x + B * y + C
        /// </summary>
        public double A { get; set; }
        public double B { get; set; }
        public double C { get; set; }

        /// <summary>
        /// Plane normal (normalized).
        /// </summary>
        public Vector3D Normal { get; set; }

        /// <summary>
        /// Centroid of the inlier points used for the fit.
        /// </summary>
        public Point3D Centroid { get; set; }

        /// <summary>
        /// Average absolute distance (in Z) from points to plane.
        /// </summary>
        public double AverageError { get; set; }

        /// <summary>
        /// The points that belong to this plane (height band).
        /// </summary>
        public List<Point3D> InlierPoints { get; set; } = new List<Point3D>();
    }

    public static class PointCloudPlaneFitting
    {
        /// <summary>
        /// Finds multiple best-fit (roughly horizontal) planes in a point cloud.
        /// It groups points by height (Z) into bands of thickness bandThickness,
        /// then fits z = a*x + b*y + c to each band via least squares.
        /// 
        /// Assumes surfaces are mostly parallel to the XY plane.
        /// </summary>
        /// <param name="points">Input point cloud.</param>
        /// <param name="bandThickness">
        /// Max Z-span per band (e.g. 0.05 for 0.05 mm if your units are mm).
        /// Points whose Z differs by more than this will go into different planes.
        /// </param>
        /// <param name="minPointsPerPlane">Minimum number of points required to fit a plane.</param>
        /// <returns>List of plane fit results (one per detected height band).</returns>
        public static List<PlaneFitResult> FitHorizontalPlanesByHeight(
            Point3DCollection points,
            double bandThickness,
            int minPointsPerPlane = 100)
        {
            var results = new List<PlaneFitResult>();
            if (points == null || points.Count == 0)
                return results;

            // 1. Sort points by Z
            var sorted = points.OrderBy(p => p.Z).ToList();

            // 2. Group into bands along Z
            var currentBand = new List<Point3D>();
            double currentBandStartZ = sorted[0].Z;

            foreach (var p in sorted)
            {
                if (currentBand.Count == 0)
                {
                    currentBand.Add(p);
                    currentBandStartZ = p.Z;
                    continue;
                }

                // If this point is still within the Z band, keep adding
                if (Math.Abs(p.Z - currentBandStartZ) <= bandThickness)
                {
                    currentBand.Add(p);
                }
                else
                {
                    // Finish current band
                    if (currentBand.Count >= minPointsPerPlane)
                    {
                        var plane = FitHorizontalPlane(currentBand);
                        if (plane != null)
                            results.Add(plane);
                    }

                    // Start new band
                    currentBand = new List<Point3D> { p };
                    currentBandStartZ = p.Z;
                }
            }

            // Last band
            if (currentBand.Count >= minPointsPerPlane)
            {
                var plane = FitHorizontalPlane(currentBand);
                if (plane != null)
                    results.Add(plane);
            }

            return results;
        }

        /// <summary>
        /// Fits a single plane z = a*x + b*y + c (least squares)
        /// to the given (roughly horizontal) surface points.
        /// Returns null if the system is degenerate.
        /// </summary>
        private static PlaneFitResult? FitHorizontalPlane(List<Point3D> pts)
        {
            int n = pts.Count;
            if (n < 3)
                return null;

            // Accumulate sums for normal equations
            double sumX = 0, sumY = 0, sumZ = 0;
            double sumX2 = 0, sumY2 = 0, sumXY = 0;
            double sumXZ = 0, sumYZ = 0;

            foreach (var p in pts)
            {
                double x = p.X;
                double y = p.Y;
                double z = p.Z;

                sumX  += x;
                sumY  += y;
                sumZ  += z;
                sumX2 += x * x;
                sumY2 += y * y;
                sumXY += x * y;
                sumXZ += x * z;
                sumYZ += y * z;
            }

            // Normal equation matrix A and RHS b for z = a*x + b*y + c:
            // [ sumX2  sumXY  sumX ] [a] = [ sumXZ ]
            // [ sumXY  sumY2  sumY ] [b]   [ sumYZ ]
            // [ sumX   sumY   n    ] [c]   [ sumZ  ]
            double a11 = sumX2, a12 = sumXY, a13 = sumX;
            double a21 = sumXY, a22 = sumY2, a23 = sumY;
            double a31 = sumX,  a32 = sumY,  a33 = n;

            double b1 = sumXZ, b2 = sumYZ, b3 = sumZ;

            // Solve via Cramer's rule (3x3)
            double detA = 
                a11 * (a22 * a33 - a23 * a32) -
                a12 * (a21 * a33 - a23 * a31) +
                a13 * (a21 * a32 - a22 * a31);

            if (Math.Abs(detA) < 1e-12)
            {
                // Degenerate system – points may be collinear or too noisy
                return null;
            }

            // Determinants for a, b, c
            double detA1 =
                b1  * (a22 * a33 - a23 * a32) -
                a12 * (b2  * a33 - a23 * b3 ) +
                a13 * (b2  * a32 - a22 * b3 );

            double detA2 =
                a11 * (b2  * a33 - a23 * b3 ) -
                b1  * (a21 * a33 - a23 * a31) +
                a13 * (a21 * b3  - b2  * a31);

            double detA3 =
                a11 * (a22 * b3  - b2  * a32) -
                a12 * (a21 * b3  - b2  * a31) +
                b1  * (a21 * a32 - a22 * a31);

            double A = detA1 / detA;
            double B = detA2 / detA;
            double C = detA3 / detA;

            // Normal of plane z - A*x - B*y - C = 0 is (A, B, -1)
            Vector3D normal = new Vector3D(A, B, -1.0);
            if (normal.Length > 0)
                normal.Normalize();

            // Centroid
            var centroid = new Point3D(sumX / n, sumY / n, sumZ / n);

            // Average absolute error in Z
            double errSum = 0;
            foreach (var p in pts)
            {
                double zFit = A * p.X + B * p.Y + C;
                errSum += Math.Abs(p.Z - zFit);
            }
            double avgErr = errSum / n;

            return new PlaneFitResult
            {
                A = A,
                B = B,
                C = C,
                Normal = normal,
                Centroid = centroid,
                AverageError = avgErr,
                InlierPoints = new List<Point3D>(pts)
            };
        }
    }
}


----

<helix:HelixViewport3D x:Name="Viewport">
    <helix:DefaultLights />

    <!-- Existing stuff -->
    <ModelVisual3D x:Name="MeshModel" />
    <helix:PointsVisual3D x:Name="PointCloudPoints"
                          Color="Red"
                          Size="1" />
    
    <!-- New: container for fitted planes -->
    <ModelVisual3D x:Name="PlanesModel" />
</helix:HelixViewport3D>


------

using HelixToolkit.Wpf;
using System.Windows.Media;
using System.Windows.Media.Media3D;
using System.Collections.Generic;

public static class PlaneVisualHelper
{
    /// <summary>
    /// Create a rectangular mesh that lies on the fitted plane and spans the
    /// inlier points' bounding box (with a padding factor).
    /// </summary>
    public static GeometryModel3D CreatePlaneGeometry(
        PlaneFitResult plane,
        double paddingFactor = 1.2)
    {
        if (plane.InlierPoints == null || plane.InlierPoints.Count < 3)
            return null;

        var centroid = plane.Centroid;
        var n = plane.Normal;
        if (n.LengthSquared < 1e-12)
            return null;

        // Choose an "up" vector that is not parallel to the normal
        Vector3D up = (Math.Abs(n.Z) < 0.9)
            ? new Vector3D(0, 0, 1)
            : new Vector3D(0, 1, 0);

        // Build orthonormal basis (u, v) in the plane
        Vector3D u = Vector3D.CrossProduct(n, up);
        if (u.LengthSquared < 1e-12)
            return null;
        u.Normalize();

        Vector3D v = Vector3D.CrossProduct(n, u);
        v.Normalize();

        // Project inlier points onto (u, v) axes to get 2D bounds
        double minU = double.MaxValue, maxU = double.MinValue;
        double minV = double.MaxValue, maxV = double.MinValue;

        foreach (var p in plane.InlierPoints)
        {
            Vector3D d = p - centroid;
            double du = Vector3D.DotProduct(d, u);
            double dv = Vector3D.DotProduct(d, v);

            if (du < minU) minU = du;
            if (du > maxU) maxU = du;
            if (dv < minV) minV = dv;
            if (dv > maxV) maxV = dv;
        }

        // Pad the rectangle a bit beyond the inlier extents
        double du = maxU - minU;
        double dv = maxV - minV;
        double padU = du * (paddingFactor - 1.0) / 2.0;
        double padV = dv * (paddingFactor - 1.0) / 2.0;

        minU -= padU; maxU += padU;
        minV -= padV; maxV += padV;

        // Corners in 3D: center + combination of (u, v)
        Point3D p00 = centroid + minU * u + minV * v;
        Point3D p10 = centroid + maxU * u + minV * v;
        Point3D p11 = centroid + maxU * u + maxV * v;
        Point3D p01 = centroid + minU * u + maxV * v;

        var mb = new MeshBuilder(false, false);
        mb.AddQuad(p00, p10, p11, p01);
        var mesh = mb.ToMesh();

        // Semi-transparent material so you can see the cloud through it
        var frontBrush = new SolidColorBrush(Color.FromArgb(80, 0, 255, 0));  // translucent green
        var backBrush  = new SolidColorBrush(Color.FromArgb(40, 0, 255, 0));

        var material   = new DiffuseMaterial(frontBrush);
        var backMat    = new DiffuseMaterial(backBrush);

        return new GeometryModel3D
        {
            Geometry    = mesh,
            Material    = material,
            BackMaterial = backMat
        };
    }
}

-------

using PointCloudUtils; // where PlaneFitResult / FitHorizontalPlanesByHeight live

private void ShowPlanesForCloud(Point3DCollection cloud)
{
    // 1. Fit planes
    double bandThickness = 0.1;   // adjust to your Z spacing (e.g. 0.1 mm)
    int minPoints = 500;

    var planes = PointCloudPlaneFitting.FitHorizontalPlanesByHeight(
        cloud,
        bandThickness,
        minPoints);

    // 2. Build a Model3DGroup with one quad per plane
    var group = new Model3DGroup();
    foreach (var plane in planes)
    {
        var gm = PlaneVisualHelper.CreatePlaneGeometry(plane);
        if (gm != null)
            group.Children.Add(gm);
    }

    // 3. Put them into the PlanesModel visual in the viewport
    PlanesModel.Content = group;

    // Optional: zoom so everything is in view
    Viewport.ZoomExtents();
}