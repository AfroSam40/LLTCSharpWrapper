// HelixToolkit.Wpf.SharpDX equivalent of your WPF BuildPlanePatchVisual()
// Creates a MeshGeometryModel3D (SharpDX) from PlaneFitResult by:
// 1) building (u,v,n) basis on the plane
// 2) projecting inliers to 2D (u,v)
// 3) convex hull in 2D (OpenCV)
// 4) optional padding (scale hull about centroid)
// 5) triangulate as a fan
//
// Assumes PlaneFitResult has: InlierPoints (IList<Point3D>), Normal (Vector3D), Centroid (Point3D)
//
// NuGet: HelixToolkit.Wpf.SharpDX + OpenCvSharp4
//
// NOTE: Avoid ToVector3() ambiguity by explicitly constructing System.Numerics.Vector3.

using System;
using System.Collections.Generic;
using System.Linq;
using System.Windows.Media.Media3D;
using HelixToolkit.Wpf.SharpDX;
using SharpDX;
using OpenCvSharp;

public static MeshGeometryModel3D BuildPlanePatchModelSharpDx(
    PlaneFitResult plane,
    double paddingFactor = 1.0,
    Color4? color = null,
    bool twoSided = true)
{
    if (plane == null) throw new ArgumentNullException(nameof(plane));
    var pts = plane.InlierPoints;
    if (pts == null || pts.Count < 3) return null;

    // ---------- build orthonormal basis (u, v, n) in WPF types ----------
    Vector3D n = plane.Normal;
    if (n.LengthSquared < 1e-12) n = new Vector3D(0, 0, 1);
    n.Normalize();

    Vector3D up = Math.Abs(n.Z) < 0.9 ? new Vector3D(0, 0, 1) : new Vector3D(1, 0, 0);

    Vector3D u = Vector3D.CrossProduct(n, up);
    if (u.LengthSquared < 1e-12) u = Vector3D.CrossProduct(n, new Vector3D(0, 1, 0));
    if (u.LengthSquared < 1e-12) u = new Vector3D(1, 0, 0);
    u.Normalize();

    Vector3D v = Vector3D.CrossProduct(n, u);
    v.Normalize();

    Point3D c = plane.Centroid;

    // ---------- project to 2D (u,v) ----------
    var proj2D = new List<System.Windows.Point>(pts.Count);
    foreach (var p in pts)
    {
        Vector3D d = p - c;
        double ux = Vector3D.DotProduct(d, u);
        double vy = Vector3D.DotProduct(d, v);
        proj2D.Add(new System.Windows.Point(ux, vy));
    }

    // ---------- convex hull in 2D using OpenCV ----------
    // Convert to OpenCV Point2f[]; ConvexHull wants Point2f (or Point) arrays
    var cvPts = proj2D.Select(p => new Point2f((float)p.X, (float)p.Y)).ToArray();
    if (cvPts.Length < 3) return null;

    Point2f[] hull;
    {
        // returnPoints=true returns hull points directly
        hull = Cv2.ConvexHull(cvPts, returnPoints: true);
    }
    if (hull == null || hull.Length < 3) return null;

    // ---------- optional padding: expand hull around its centroid ----------
    if (paddingFactor > 1.0)
    {
        float cx = hull.Average(p => p.X);
        float cy = hull.Average(p => p.Y);
        for (int i = 0; i < hull.Length; i++)
        {
            float dx = hull[i].X - cx;
            float dy = hull[i].Y - cy;
            hull[i] = new Point2f(cx + dx * (float)paddingFactor, cy + dy * (float)paddingFactor);
        }
    }

    // ---------- build SharpDX mesh ----------
    var positions = new Vector3Collection();
    positions.Capacity = hull.Length;

    // Convert 2D hull back to 3D points on the plane: P = c + ux*u + vy*v
    for (int i = 0; i < hull.Length; i++)
    {
        var h = hull[i];

        Point3D wp = c + (double)h.X * u + (double)h.Y * v;

        // IMPORTANT: Use System.Numerics.Vector3 explicitly (HelixToolkit.Wpf.SharpDX uses SharpDX.Vector3)
        positions.Add(new Vector3((float)wp.X, (float)wp.Y, (float)wp.Z));
    }

    int nVerts = positions.Count;
    if (nVerts < 3) return null;

    var indices = new IntCollection();
    // Triangle fan: (0, i, i+1)
    for (int i = 1; i < nVerts - 1; i++)
    {
        indices.Add(0);
        indices.Add(i);
        indices.Add(i + 1);
    }

    // Provide normals (recommended; Helix can compute in some pipelines, but don’t rely on it)
    var nn = new Vector3((float)n.X, (float)n.Y, (float)n.Z);
    var normals = new Vector3Collection();
    normals.Capacity = nVerts;
    for (int i = 0; i < nVerts; i++) normals.Add(nn);

    var mesh = new HelixToolkit.Wpf.SharpDX.MeshGeometry3D
    {
        Positions = positions,
        Indices = indices,
        Normals = normals
    };

    // ---------- material ----------
    var c4 = color ?? new Color4(0.78f, 0.50f, 1.0f, 0.30f); // default semi-transparent purple-ish
    var mat = new PhongMaterial
    {
        DiffuseColor = c4,
        AmbientColor = new Color4(c4.Red * 0.2f, c4.Green * 0.2f, c4.Blue * 0.2f, c4.Alpha),
        SpecularColor = new Color4(0, 0, 0, c4.Alpha),
        Shininess = 1f
    };

    return new MeshGeometryModel3D
    {
        Geometry = mesh,
        Material = mat,
        CullMode = twoSided ? SharpDX.Direct3D11.CullMode.None : SharpDX.Direct3D11.CullMode.Back,
        IsTransparent = c4.Alpha < 1f
    };
}
```0