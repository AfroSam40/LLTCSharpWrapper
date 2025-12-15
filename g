using System;
using System.Collections.Generic;
using System.Linq;
using System.Windows.Media;
using System.Windows.Media.Media3D;
using HelixToolkit.Wpf;

public struct PlaneFitResult
{
    public Vector3D Normal;
    public Point3D Centroid;
    // You can add RMSE or other fields here if you already use them.
}

public class BlobSlice
{
    /// <summary>Bottom of slice (height along plane normal).</summary>
    public double H0 { get; set; }

    /// <summary>Top of slice (height along plane normal).</summary>
    public double H1 { get; set; }

    /// <summary>Center height (midpoint between H0 and H1).</summary>
    public double HCenter { get; set; }

    /// <summary>World-space center point of the slice (center of cross-section).</summary>
    public Point3D CenterWorld { get; set; }

    /// <summary>Plane normal (same for all slices of a given blob).</summary>
    public Vector3D Normal { get; set; }

    /// <summary>Equivalent radius of the cross-section (assuming circular).</summary>
    public double Radius { get; set; }

    /// <summary>Cross-section area (π r²).</summary>
    public double Area { get; set; }

    /// <summary>Number of points used for this slice.</summary>
    public int PointCount { get; set; }
}

-------

public static class VolumeHelpers
{
    /// <summary>
    /// Estimate volume of a blob sitting on a plane using slice-by-slice
    /// circular cross sections (method of disks), and also return slice info.
    /// </summary>
    /// <param name="points">Isolated blob point cloud.</param>
    /// <param name="basePlane">Plane the blob sits on.</param>
    /// <param name="sliceThickness">Δh (same units as your coordinates, e.g. mm).</param>
    /// <param name="slices">Output: list of slice descriptors.</param>
    /// <param name="minPointsPerSlice">Minimum points required to accept a slice.</param>
    /// <returns>Estimated volume in cubic units of your coordinate system.</returns>
    public static double EstimateBlobVolumeBySlices(
        Point3DCollection points,
        PlaneFitResult basePlane,
        double sliceThickness,
        out List<BlobSlice> slices,
        int minPointsPerSlice = 50)
    {
        slices = new List<BlobSlice>();

        if (points == null || points.Count == 0)
            return 0.0;
        if (sliceThickness <= 0)
            throw new ArgumentOutOfRangeException(nameof(sliceThickness));

        // ---- 1. Orthonormal basis (u, v, n) ----
        Vector3D n = basePlane.Normal;
        if (n.LengthSquared < 1e-12)
            throw new ArgumentException("Plane normal is zero.", nameof(basePlane));
        n.Normalize();

        // Choose a helper vector not parallel to n
        Vector3D temp = Math.Abs(n.Z) < 0.9
            ? new Vector3D(0, 0, 1)
            : new Vector3D(1, 0, 0);

        // u = normalized (temp × n), v = n × u
        Vector3D u = Vector3D.CrossProduct(temp, n);
        if (u.LengthSquared < 1e-12)
            u = new Vector3D(1, 0, 0);
        u.Normalize();

        Vector3D v = Vector3D.CrossProduct(n, u);
        v.Normalize();

        Point3D origin = basePlane.Centroid;

        // ---- 2. Transform points into local (u, v, h) coords ----
        var localPoints = new List<(double U, double V, double H)>(points.Count);
        foreach (var p in points)
        {
            Vector3D d = p - origin;
            double h = Vector3D.DotProduct(d, n);   // height above plane
            double uu = Vector3D.DotProduct(d, u);  // in-plane coord
            double vv = Vector3D.DotProduct(d, v);

            // Ignore tiny negative heights (numerical noise)
            if (h >= -1e-6)
                localPoints.Add((uu, vv, h));
        }

        if (localPoints.Count == 0)
            return 0.0;

        double minH = localPoints.Min(p => p.H);
        double maxH = localPoints.Max(p => p.H);

        if (minH < 0) minH = 0;

        double volume = 0.0;

        // ---- 3. Walk slices along height ----
        for (double h0 = minH; h0 < maxH; h0 += sliceThickness)
        {
            double h1 = h0 + sliceThickness;

            // Points in this slice [h0, h1)
            var slice = localPoints
                .Where(p => p.H >= h0 && p.H < h1)
                .ToList();

            if (slice.Count < minPointsPerSlice)
                continue;

            // ---- 4. Equivalent circular cross-section ----
            // centroid in (u, v)
            double cu = slice.Average(p => p.U);
            double cv = slice.Average(p => p.V);

            double rSum = 0.0;
            foreach (var s in slice)
            {
                double du = s.U - cu;
                double dv = s.V - cv;
                rSum += Math.Sqrt(du * du + dv * dv);
            }

            double r = rSum / slice.Count;
            if (r <= 0) continue;

            double area = Math.PI * r * r;
            double dH = sliceThickness;

            volume += area * dH;

            // World-space center of this slice
            double hCenter = 0.5 * (h0 + h1);
            Vector3D offset =
                (u * cu) +
                (v * cv) +
                (n * hCenter);

            Point3D centerWorld = origin + offset;

            slices.Add(new BlobSlice
            {
                H0 = h0,
                H1 = h1,
                HCenter = hCenter,
                CenterWorld = centerWorld,
                Normal = n,
                Radius = r,
                Area = area,
                PointCount = slice.Count
            });
        }

        return volume;
    }
}



-------

public static class SliceVisualizationHelpers
{
    /// <summary>
    /// Renders each slice as a thin transparent cylinder inside the point cloud.
    /// </summary>
    /// <param name="target">
    /// A ModelVisual3D to host the slice geometry (e.g. x:Name="SliceVisual").
    /// </param>
    /// <param name="slices">Slice info returned from EstimateBlobVolumeBySlices.</param>
    /// <param name="thicknessScale">
    /// Factor for slice thickness relative to slice height (0..1).
    /// </param>
    /// <param name="thetaDiv">Number of segments around circle (16–64 is typical).</param>
    public static void ShowSlicesAsCylinders(
        ModelVisual3D target,
        IEnumerable<BlobSlice> slices,
        double thicknessScale = 0.7,
        int thetaDiv = 48)
    {
        if (target == null) throw new ArgumentNullException(nameof(target));

        target.Children.Clear();

        var group = new Model3DGroup();

        // Transparent red
        var color = Color.FromArgb(80, 255, 0, 0);
        var material = MaterialHelper.CreateMaterial(color);

        foreach (var s in slices)
        {
            if (s.Radius <= 0 || s.PointCount <= 0)
                continue;

            Vector3D axis = s.Normal;
            if (axis.LengthSquared < 1e-12)
                axis = new Vector3D(0, 0, 1);
            axis.Normalize();

            double sliceHeight = Math.Max(s.H1 - s.H0, 1e-6);
            double thickness = sliceHeight * thicknessScale;
            if (thickness <= 0) thickness = sliceHeight;

            Vector3D half = axis * (thickness / 2.0);

            Point3D p0 = s.CenterWorld - half;
            Point3D p1 = s.CenterWorld + half;

            var mb = new MeshBuilder(false, false);
            mb.AddCylinder(p0, p1, s.Radius, thetaDiv);
            var mesh = mb.ToMesh();

            var geom = new GeometryModel3D
            {
                Geometry = mesh,
                Material = material,
                BackMaterial = material
            };

            group.Children.Add(geom);
        }

        target.Content = group;
    }
}