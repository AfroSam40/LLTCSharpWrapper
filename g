using System;
using System.Linq;
using System.Windows.Media.Media3D;
using HelixToolkit.Wpf.SharpDX;
using SharpDX;

public static class PointCloudSharpDxHelpers
{
    /// <summary>
    /// Builds a PointGeometry3D from a WPF Point3DCollection and assigns a Z-heatmap color per point.
    /// Returns the geometry (Positions + Colors) ready to be assigned to a PointGeometryModel3D.Geometry.
    ///
    /// Colors is a Color4Collection (as required by HelixToolkit.Wpf.SharpDX).
    /// </summary>
    public static PointGeometry3D BuildHeatmapGeometryFromPoints(
        Point3DCollection points,
        bool invert = false,
        float uniformAlpha = 1f)
    {
        if (points == null) throw new ArgumentNullException(nameof(points));
        if (points.Count == 0) return new PointGeometry3D();

        if (uniformAlpha < 0f) uniformAlpha = 0f;
        if (uniformAlpha > 1f) uniformAlpha = 1f;

        // Build positions (Vector3Collection)
        var positions = new Vector3Collection(points.Count);

        // Compute Z range in one pass
        double zMin = double.PositiveInfinity;
        double zMax = double.NegativeInfinity;

        for (int i = 0; i < points.Count; i++)
        {
            var p = points[i];
            positions.Add(new Vector3((float)p.X, (float)p.Y, (float)p.Z));

            if (p.Z < zMin) zMin = p.Z;
            if (p.Z > zMax) zMax = p.Z;
        }

        double range = zMax - zMin;
        if (range <= 1e-12) range = 1.0;

        // Build colors (Color4Collection)
        var colors = new Color4Collection(points.Count);

        for (int i = 0; i < points.Count; i++)
        {
            double t = (points[i].Z - zMin) / range; // 0..1
            if (invert) t = 1.0 - t;

            var c = HeatColor((float)t, uniformAlpha);
            colors.Add(c);
        }

        return new PointGeometry3D
        {
            Positions = positions,
            Colors = colors
        };
    }

    /// <summary>
    /// Simple heatmap colormap: blue -> cyan -> green -> yellow -> red.
    /// </summary>
    private static Color4 HeatColor(float t, float a)
    {
        t = MathUtil.Clamp(t, 0f, 1f);

        // 0.00-0.25: blue -> cyan
        if (t < 0.25f)
        {
            float u = t / 0.25f;
            return new Color4(0f, u, 1f, a);
        }
        // 0.25-0.50: cyan -> green
        if (t < 0.50f)
        {
            float u = (t - 0.25f) / 0.25f;
            return new Color4(0f, 1f, 1f - u, a);
        }
        // 0.50-0.75: green -> yellow
        if (t < 0.75f)
        {
            float u = (t - 0.50f) / 0.25f;
            return new Color4(u, 1f, 0f, a);
        }
        // 0.75-1.00: yellow -> red
        float v = (t - 0.75f) / 0.25f;
        return new Color4(1f, 1f - v, 0f, a);
    }
}