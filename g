using System.Windows.Media;
using System.Windows.Media.Media3D;

public static class PointCloudProcessing
{
    /// <summary>
    /// Build a 3D model consisting of thick “puck” slices for a Hershey-kiss blob.
    /// Each slice is a short cylinder centered at BlobSlice.CenterWorld with
    /// axis = BlobSlice.Normal and radius = BlobSlice.Radius.
    /// </summary>
    /// <param name="slices">Blob slices from EstimateBlobVolumeBySlices.</param>
    /// <param name="visualThickness">
    /// Thickness of each slice in world units (purely visual; 
    /// volume computation still comes from your numeric routine).
    /// </param>
    /// <param name="angleStepDegrees">
    /// Angular step for polygonizing the circle (smaller = smoother, heavier).
    /// </param>
    public static Model3DGroup BuildBlobSlicesModelWithThickness(
        IEnumerable<BlobSlice> slices,
        double visualThickness,
        double angleStepDegrees = 15.0)
    {
        if (slices == null) throw new ArgumentNullException(nameof(slices));
        if (visualThickness <= 0) throw new ArgumentOutOfRangeException(nameof(visualThickness));
        if (angleStepDegrees <= 0 || angleStepDegrees > 180)
            throw new ArgumentOutOfRangeException(nameof(angleStepDegrees));

        var group = new Model3DGroup();

        foreach (var s in slices)
        {
            if (s == null) continue;
            if (s.Radius <= 0) continue;
            if (s.PointCount <= 0) continue;

            var gm = BuildSlicePuckGeometry(s, visualThickness, angleStepDegrees);
            if (gm != null)
                group.Children.Add(gm);
        }

        return group;
    }

    /// <summary>
    /// Build a single thick disc (short cylinder) for one slice.
    /// </summary>
    private static GeometryModel3D? BuildSlicePuckGeometry(
        BlobSlice slice,
        double thickness,
        double angleStepDegrees)
    {
        Vector3D n = slice.Normal;
        if (n.LengthSquared < 1e-12)
            return null;

        n.Normalize();
        double halfT = thickness / 2.0;
        double radius = slice.Radius;
        Point3D center = slice.CenterWorld;

        // ---- 1. Build local orthonormal basis (u, v, n) ----
        Vector3D temp = Math.Abs(n.Z) < 0.9
            ? new Vector3D(0, 0, 1)
            : new Vector3D(1, 0, 0);

        Vector3D u = Vector3D.CrossProduct(temp, n);
        if (u.LengthSquared < 1e-12)
            return null;
        u.Normalize();

        Vector3D v = Vector3D.CrossProduct(n, u);
        v.Normalize();

        // ---- 2. Prepare collections ----
        var positions = new Point3DCollection();
        var triangleIndices = new Int32Collection();
        var normals = new Vector3DCollection();

        // Colors: translucent red
        var frontBrush = new SolidColorBrush(Color.FromArgb(96, 255, 0, 0));
        var backBrush  = new SolidColorBrush(Color.FromArgb(96, 255, 0, 0));
        frontBrush.Freeze();
        backBrush.Freeze();

        // ---- 3. Centers ----
        Point3D topCenter = center + n * halfT;
        Point3D bottomCenter = center - n * halfT;

        int idxTopCenter = positions.Count;
        positions.Add(topCenter);
        normals.Add(n);

        int idxBottomCenter = positions.Count;
        positions.Add(bottomCenter);
        normals.Add(-n);

        // ---- 4. Rings ----
        int steps = (int)Math.Round(360.0 / angleStepDegrees);
        if (steps < 3) steps = 3;

        // Store indices of ring vertices
        int[] topRing = new int[steps];
        int[] bottomRing = new int[steps];

        double radiansStep = Math.PI * 2.0 / steps;

        for (int i = 0; i < steps; i++)
        {
            double angle = i * radiansStep;
            // radial direction in the plane
            Vector3D dir = Math.Cos(angle) * u + Math.Sin(angle) * v;
            dir.Normalize();

            Point3D topPt    = topCenter    + dir * radius;
            Point3D bottomPt = bottomCenter + dir * radius;

            int idxTop = positions.Count;
            positions.Add(topPt);
            // For nicer shading on sides, use outward radial normal for ring verts
            normals.Add(dir);

            int idxBottom = positions.Count;
            positions.Add(bottomPt);
            normals.Add(dir);

            topRing[i] = idxTop;
            bottomRing[i] = idxBottom;
        }

        // ---- 5. Top & bottom caps ----
        for (int i = 0; i < steps; i++)
        {
            int next = (i + 1) % steps;

            int iTop      = topRing[i];
            int iTopNext  = topRing[next];
            int iBottom   = bottomRing[i];
            int iBottomNx = bottomRing[next];

            // Top cap (winding so normal ~ +n)
            triangleIndices.Add(idxTopCenter);
            triangleIndices.Add(iTop);
            triangleIndices.Add(iTopNext);

            // Bottom cap (winding so normal ~ -n)
            triangleIndices.Add(idxBottomCenter);
            triangleIndices.Add(iBottomNx);
            triangleIndices.Add(iBottom);
        }

        // ---- 6. Side walls ----
        for (int i = 0; i < steps; i++)
        {
            int next = (i + 1) % steps;

            int iTop      = topRing[i];
            int iTopNext  = topRing[next];
            int iBottom   = bottomRing[i];
            int iBottomNx = bottomRing[next];

            // Quad = (top_i, bottom_i, bottom_next, top_next)
            // Triangle 1
            triangleIndices.Add(iTop);
            triangleIndices.Add(iBottom);
            triangleIndices.Add(iBottomNx);

            // Triangle 2
            triangleIndices.Add(iTop);
            triangleIndices.Add(iBottomNx);
            triangleIndices.Add(iTopNext);
        }

        var mesh = new MeshGeometry3D
        {
            Positions = positions,
            TriangleIndices = triangleIndices,
            Normals = normals
        };

        var material = new DiffuseMaterial(frontBrush);
        var backMat  = new DiffuseMaterial(backBrush);

        return new GeometryModel3D
        {
            Geometry = mesh,
            Material = material,
            BackMaterial = backMat
        };
    }
}