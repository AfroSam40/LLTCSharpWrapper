using System;
using System.Linq;
using HelixToolkit.Wpf.SharpDX;
using SharpDX;

// Colors points by height (Z) to create a heatmap, and writes Colors back into the point model.
// Assumes your point cloud is rendered by a PointGeometryModel3D inside the Viewport3DX.
// If you have multiple point models, it will use the first one it finds (you can easily change that).
public static void ApplyZHeatmapToPointCloud(Viewport3DX viewport, bool invert = false)
{
    if (viewport == null) throw new ArgumentNullException(nameof(viewport));

    // Find the first PointGeometryModel3D in the viewport
    var pointModel = viewport.Items
        .OfType<PointGeometryModel3D>()
        .FirstOrDefault();

    if (pointModel == null)
        throw new InvalidOperationException("No PointGeometryModel3D found in Viewport3DX.Items.");

    if (pointModel.Geometry is not PointGeometry3D geo)
        throw new InvalidOperationException("PointGeometryModel3D.Geometry is not a PointGeometry3D.");

    var positions = geo.Positions;
    if (positions == null || positions.Count == 0)
        throw new InvalidOperationException("PointGeometry3D.Positions is empty.");

    // Compute Z range
    float zMin = float.PositiveInfinity;
    float zMax = float.NegativeInfinity;

    for (int i = 0; i < positions.Count; i++)
    {
        float z = positions[i].Z;
        if (z < zMin) zMin = z;
        if (z > zMax) zMax = z;
    }

    float range = zMax - zMin;
    if (range <= 1e-12f) range = 1f; // avoid div-by-zero if all points share same Z

    // Allocate color array
    // HelixToolkit.SharpDX uses SharpDX.Color4 (RGBA floats 0..1)
    var colors = new Color4[positions.Count];

    // Simple heatmap: blue -> cyan -> green -> yellow -> red
    // You can swap this for a better colormap later (Turbo/Viridis/etc).
    for (int i = 0; i < positions.Count; i++)
    {
        float t = (positions[i].Z - zMin) / range; // 0..1
        if (invert) t = 1f - t;

        colors[i] = HeatColor(t);
    }

    // Write colors back to geometry
    // (Helix will re-upload to GPU when geometry changes)
    geo.Colors = colors;

    // Re-assigning ensures binding/refresh if you swap Geometry objects elsewhere
    pointModel.Geometry = geo;

    // Local helper: piecewise gradient
    static Color4 HeatColor(float t)
    {
        t = MathUtil.Clamp(t, 0f, 1f);

        // 0.00-0.25: blue -> cyan
        if (t < 0.25f)
        {
            float u = t / 0.25f;
            return new Color4(0f, u, 1f, 1f);
        }
        // 0.25-0.50: cyan -> green
        if (t < 0.50f)
        {
            float u = (t - 0.25f) / 0.25f;
            return new Color4(0f, 1f, 1f - u, 1f);
        }
        // 0.50-0.75: green -> yellow
        if (t < 0.75f)
        {
            float u = (t - 0.50f) / 0.25f;
            return new Color4(u, 1f, 0f, 1f);
        }
        // 0.75-1.00: yellow -> red
        {
            float u = (t - 0.75f) / 0.25f;
            return new Color4(1f, 1f - u, 0f, 1f);
        }
    }
}