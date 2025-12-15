public static void BuildHullSliceVisuals(
    ModelVisual3D parent,
    IEnumerable<BlobSlice> slices,
    Color color,
    double opacity = 0.3,
    double thicknessScale = 1.0)   // 1.0 = true thickness, >1 exaggerates
{
    if (parent == null) throw new ArgumentNullException(nameof(parent));
    if (slices == null) throw new ArgumentNullException(nameof(slices));

    parent.Children.Clear();

    byte a = (byte)(Math.Max(0, Math.Min(1, opacity)) * 255);
    var brush = new SolidColorBrush(Color.FromArgb(a, color.R, color.G, color.B));
    brush.Freeze();

    var material     = new DiffuseMaterial(brush);
    var backMaterial = material;

    foreach (var slice in slices)
    {
        var hull = slice.HullWorld;
        if (hull == null || hull.Length < 3)
            continue;

        // --- 1. Compute thickness & offsets along normal ---
        Vector3D n = slice.Normal;
        if (n.LengthSquared < 1e-12)
            continue;
        n.Normalize();

        double h0 = slice.H0;
        double h1 = slice.H1;
        double thickness = (h1 - h0) * thicknessScale;
        if (thickness <= 0)
            continue;

        double halfT = 0.5 * thickness;
        Vector3D offset = n * halfT;

        // Bottom & top centers
        Point3D centerMid    = slice.CenterWorld;
        Point3D centerBottom = centerMid - offset;
        Point3D centerTop    = centerMid + offset;

        int nHull = hull.Length;

        var positions = new Point3DCollection();
        var indices   = new Int32Collection();

        // Index layout:
        //  0              : bottom center
        //  1 .. nHull     : bottom ring
        //  nHull+1        : top center
        //  nHull+2 .. end : top ring

        // --- 2. Add bottom center + ring ---
        positions.Add(centerBottom);          // index 0
        for (int i = 0; i < nHull; i++)
            positions.Add(hull[i] - offset);  // 1..nHull

        int bottomCenter = 0;
        int bottomStart  = 1;

        // Bottom cap (fan)
        for (int i = 0; i < nHull; i++)
        {
            int i2 = bottomStart + i;
            int i3 = bottomStart + ((i + 1) % nHull);
            indices.Add(bottomCenter);
            indices.Add(i2);
            indices.Add(i3);
        }

        // --- 3. Add top center + ring ---
        int topCenter = positions.Count;      // nHull+1
        positions.Add(centerTop);

        int topStart = positions.Count;       // nHull+2
        for (int i = 0; i < nHull; i++)
            positions.Add(hull[i] + offset);

        // Top cap (fan) – flip winding so outward normals stay consistent
        for (int i = 0; i < nHull; i++)
        {
            int i2 = topStart + i;
            int i3 = topStart + ((i + 1) % nHull);
            indices.Add(topCenter);
            indices.Add(i3);
            indices.Add(i2);
        }

        // --- 4. Side quads between bottom & top rings ---
        for (int i = 0; i < nHull; i++)
        {
            int bi0 = bottomStart + i;
            int bi1 = bottomStart + ((i + 1) % nHull);
            int ti0 = topStart + i;
            int ti1 = topStart + ((i + 1) % nHull);

            // Quad = (bi0, bi1, ti1, ti0)
            // Triangle 1
            indices.Add(bi0);
            indices.Add(bi1);
            indices.Add(ti1);
            // Triangle 2
            indices.Add(bi0);
            indices.Add(ti1);
            indices.Add(ti0);
        }

        var mesh = new MeshGeometry3D
        {
            Positions      = positions,
            TriangleIndices = indices
        };
        mesh.Freeze();

        var geom = new GeometryModel3D
        {
            Geometry     = mesh,
            Material     = material,
            BackMaterial = backMaterial
        };

        parent.Children.Add(new ModelVisual3D { Content = geom });
    }
}