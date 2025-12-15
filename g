using System;
using System.Collections.Generic;
using System.Windows.Media;
using System.Windows.Media.Media3D;
using HelixToolkit.Wpf;

public static class SliceVisualizationHelpers
{
    // Reuse a single unit cylinder mesh for all slices
    private static MeshGeometry3D? _unitCylinderMesh;

    /// <summary>
    /// Show slice cross-sections as translucent cylinders inside the given root visual.
    /// Does NOT use MeshBuilder; only MeshGeometry3D + transforms.
    /// </summary>
    public static void ShowSlicesAsCylinders(
        ModelVisual3D rootVisual,
        IList<BlobSlice> slices,
        PlaneFitResult plane,
        Color color,
        double thicknessScale = 1.0,
        int thetaDiv = 32)
    {
        if (rootVisual == null) throw new ArgumentNullException(nameof(rootVisual));
        if (slices == null) throw new ArgumentNullException(nameof(slices));
        if (slices.Count == 0) return;

        // Lazily create a reusable unit cylinder mesh:
        // radius = 1, height = 1, centered at origin, axis along +Z
        if (_unitCylinderMesh == null)
            _unitCylinderMesh = CreateUnitCylinderMesh(thetaDiv);

        // Build material (transparent)
        var brush = new SolidColorBrush(Color.FromArgb(80, color.R, color.G, color.B));
        brush.Freeze();
        var material = new DiffuseMaterial(brush);

        var group = new Model3DGroup();

        Vector3D n = plane.Normal;
        if (n.LengthSquared < 1e-12)
            n = new Vector3D(0, 0, 1); // fallback

        n.Normalize();

        foreach (var s in slices)
        {
            double radius = s.Radius;
            if (radius <= 0) continue;

            double baseThickness = Math.Max(Math.Abs(s.Z1 - s.Z0), 1e-6);
            double thickness = baseThickness * thicknessScale;

            var transform = BuildSliceTransform(
                center: s.Center,
                normal: n,
                radius: radius,
                thickness: thickness);

            var gm = new GeometryModel3D
            {
                Geometry = _unitCylinderMesh,
                Material = material,
                BackMaterial = material,
                Transform = transform
            };

            group.Children.Add(gm);
        }

        // Add slice visuals as a child under the root visual
        var sliceVisual = new ModelVisual3D { Content = group };
        rootVisual.Children.Add(sliceVisual);
    }

    /// <summary>
    /// Creates a unit cylinder mesh (radius=1, height=1) aligned with +Z, centered at origin.
    /// Uses only MeshGeometry3D (no MeshBuilder).
    /// </summary>
    private static MeshGeometry3D CreateUnitCylinderMesh(int thetaDiv)
    {
        if (thetaDiv < 3) thetaDiv = 3;

        var mesh = new MeshGeometry3D();
        var positions = new Point3DCollection();
        var indices = new Int32Collection();

        double halfH = 0.5;

        // Rings: bottom and top
        for (int i = 0; i < thetaDiv; i++)
        {
            double angle = 2.0 * Math.PI * i / thetaDiv;
            double x = Math.Cos(angle);
            double y = Math.Sin(angle);

            // bottom ring
            positions.Add(new Point3D(x, y, -halfH));
            // top ring
            positions.Add(new Point3D(x, y, +halfH));
        }

        int bottomCenterIndex = positions.Count;
        positions.Add(new Point3D(0, 0, -halfH));

        int topCenterIndex = positions.Count;
        positions.Add(new Point3D(0, 0, +halfH));

        // Side faces
        for (int i = 0; i < thetaDiv; i++)
        {
            int j = (i + 1) % thetaDiv;

            int iBottom0 = 2 * i;
            int iTop0 = 2 * i + 1;
            int iBottom1 = 2 * j;
            int iTop1 = 2 * j + 1;

            // quad = (bottom0, bottom1, top1, top0)
            // triangle 1
            indices.Add(iBottom0);
            indices.Add(iBottom1);
            indices.Add(iTop1);

            // triangle 2
            indices.Add(iBottom0);
            indices.Add(iTop1);
            indices.Add(iTop0);
        }

        // Bottom cap (fan)
        for (int i = 0; i < thetaDiv; i++)
        {
            int j = (i + 1) % thetaDiv;
            int iBottom0 = 2 * i;
            int iBottom1 = 2 * j;

            // wind so that normal points -Z or +Z depending on your convention
            indices.Add(bottomCenterIndex);
            indices.Add(iBottom1);
            indices.Add(iBottom0);
        }

        // Top cap (fan)
        for (int i = 0; i < thetaDiv; i++)
        {
            int j = (i + 1) % thetaDiv;
            int iTop0 = 2 * i + 1;
            int iTop1 = 2 * j + 1;

            indices.Add(topCenterIndex);
            indices.Add(iTop0);
            indices.Add(iTop1);
        }

        mesh.Positions = positions;
        mesh.TriangleIndices = indices;
        // Normals/texture coords omitted; WPF can auto-generate normals if needed.

        return mesh;
    }

    /// <summary>
    /// Builds a Transform3D that takes the unit cylinder mesh (radius=1, height=1 along +Z)
    /// into world space: scaled to (radius, radius, thickness) and oriented with 'normal',
    /// centered at 'center'.
    /// </summary>
    private static Transform3D BuildSliceTransform(
        Point3D center,
        Vector3D normal,
        double radius,
        double thickness)
    {
        // Ensure normal is unit
        if (normal.LengthSquared < 1e-12)
            normal = new Vector3D(0, 0, 1);
        normal.Normalize();

        // Build orthonormal basis (u, v, n)
        Vector3D temp = Math.Abs(normal.Z) < 0.9
            ? new Vector3D(0, 0, 1)
            : new Vector3D(1, 0, 0);

        Vector3D u = Vector3D.CrossProduct(temp, normal);
        if (u.LengthSquared < 1e-12)
            u = new Vector3D(1, 0, 0);
        u.Normalize();

        Vector3D v = Vector3D.CrossProduct(normal, u);
        v.Normalize();

        // Scale basis vectors by radius / thickness
        u *= radius;
        v *= radius;
        normal *= thickness;

        // Matrix3D using column vectors u, v, n and translation center
        var m = new Matrix3D
        {
            M11 = u.X, M12 = v.X, M13 = normal.X, M14 = center.X,
            M21 = u.Y, M22 = v.Y, M23 = normal.Y, M24 = center.Y,
            M31 = u.Z, M32 = v.Z, M33 = normal.Z, M34 = center.Z,
            M44 = 1.0
        };

        return new MatrixTransform3D(m);
    }
}