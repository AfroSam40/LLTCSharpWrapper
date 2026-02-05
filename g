using System;
using System.Collections.Generic;
using System.Linq;
using System.Windows.Media.Media3D;
using HelixToolkit.Wpf.SharpDX;
using SharpDX;
using SharpDX.Direct3D11;

public static class SharpDxPlaneBuilder
{
    public static GroupModel3D CreatePlanesGroup(
        List<PlaneFitResult> fits,
        Color4 color,
        double padding = 0.0,
        double trimFraction = 0.02,
        bool twoSided = true)
    {
        var group = new GroupModel3D();
        UpdatePlanesGroup(group, fits, color, padding, trimFraction, twoSided);
        return group;
    }

    public static void UpdatePlanesGroup(
        GroupModel3D targetGroup,
        List<PlaneFitResult> fits,
        Color4 color,
        double padding = 0.0,
        double trimFraction = 0.02,
        bool twoSided = true)
    {
        if (targetGroup == null) throw new ArgumentNullException(nameof(targetGroup));

        targetGroup.Children.Clear();
        if (fits == null || fits.Count == 0) return;

        foreach (var fit in fits)
        {
            var model = CreatePlaneModelFromFit(fit, color, padding, trimFraction, twoSided);
            if (model != null)
                targetGroup.Children.Add(model);
        }
    }

    public static MeshGeometryModel3D? CreatePlaneModelFromFit(
        PlaneFitResult fit,
        Color4 color,
        double padding = 0.0,
        double trimFraction = 0.02,
        bool twoSided = true)
    {
        if (fit == null) throw new ArgumentNullException(nameof(fit));
        if (fit.InlierPoints == null || fit.InlierPoints.Count < 3) return null;

        // Normal
        Vector3D n = fit.Normal;
        if (n.LengthSquared < 1e-20)
            n = new Vector3D(-fit.A, -fit.B, 1.0);
        n.Normalize();

        // Basis U,V
        Vector3D refAxis = Math.Abs(n.Z) < 0.9 ? new Vector3D(0, 0, 1) : new Vector3D(0, 1, 0);
        Vector3D u = Vector3D.CrossProduct(refAxis, n);
        if (u.LengthSquared < 1e-20)
            u = Vector3D.CrossProduct(new Vector3D(1, 0, 0), n);
        u.Normalize();

        Vector3D v = Vector3D.CrossProduct(n, u);
        v.Normalize();

        // Point on plane near surface (centroid projected to plane z = Ax + By + C)
        double cx = fit.Centroid.X;
        double cy = fit.Centroid.Y;
        double cz = fit.A * cx + fit.B * cy + fit.C;
        Point3D p0 = new Point3D(cx, cy, cz);

        // Project inliers to UV
        int count = fit.InlierPoints.Count;
        var us = new double[count];
        var vs = new double[count];

        for (int i = 0; i < count; i++)
        {
            var p = fit.InlierPoints[i];
            Vector3D w = p - p0;
            us[i] = Vector3D.DotProduct(w, u);
            vs[i] = Vector3D.DotProduct(w, v);
        }

        GetTrimmedMinMax(us, trimFraction, out double uMin, out double uMax);
        GetTrimmedMinMax(vs, trimFraction, out double vMin, out double vMax);

        uMin -= padding; uMax += padding;
        vMin -= padding; vMax += padding;

        if ((uMax - uMin) < 1e-12 || (vMax - vMin) < 1e-12) return null;

        // Quad corners
        Point3D c00 = p0 + u * uMin + v * vMin;
        Point3D c10 = p0 + u * uMax + v * vMin;
        Point3D c11 = p0 + u * uMax + v * vMax;
        Point3D c01 = p0 + u * uMin + v * vMax;

        // Mesh (two triangles)
        var positions = new Vector3Collection
        {
            new Vector3((float)c00.X, (float)c00.Y, (float)c00.Z),
            new Vector3((float)c10.X, (float)c10.Y, (float)c10.Z),
            new Vector3((float)c11.X, (float)c11.Y, (float)c11.Z),
            new Vector3((float)c01.X, (float)c01.Y, (float)c01.Z),
        };

        var indices = new IntCollection { 0, 1, 2, 0, 2, 3 };

        var nn = new Vector3((float)n.X, (float)n.Y, (float)n.Z);
        var normals = new Vector3Collection { nn, nn, nn, nn };

        var mesh = new HelixToolkit.Wpf.SharpDX.MeshGeometry3D
        {
            Positions = positions,
            Indices = indices,
            Normals = normals
        };

        var mat = new PhongMaterial
        {
            DiffuseColor = color,
            AmbientColor = color * 0.2f
        };

        return new MeshGeometryModel3D
        {
            Geometry = mesh,
            Material = mat,
            CullMode = twoSided ? CullMode.None : CullMode.Back
        };
    }

    private static void GetTrimmedMinMax(double[] arr, double trim, out double mn, out double mx)
    {
        if (arr == null || arr.Length == 0) { mn = mx = 0; return; }
        trim = Math.Max(0.0, Math.Min(0.49, trim));

        var tmp = (double[])arr.Clone();
        Array.Sort(tmp);

        int lo = (int)Math.Floor(trim * (tmp.Length - 1));
        int hi = (int)Math.Ceiling((1.0 - trim) * (tmp.Length - 1));

        lo = Math.Max(0, Math.Min(tmp.Length - 1, lo));
        hi = Math.Max(0, Math.Min(tmp.Length - 1, hi));
        if (hi < lo) (lo, hi) = (hi, lo);

        mn = tmp[lo];
        mx = tmp[hi];
    }
}