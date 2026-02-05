using System;
using HelixToolkit.Wpf.SharpDX;
using SharpDX; // Vector3
using HelixToolkit.SharpDX.Core; // (usually safe; depends on your version)
using HelixToolkit.Wpf.SharpDX.Model; // Camera types

public static class DxCameraHelpers
{
    public static void FramePointsInView(Viewport3DX viewport, PointGeometry3D geo, float padding = 1.2f)
    {
        if (viewport == null) throw new ArgumentNullException(nameof(viewport));
        if (geo?.Positions == null || geo.Positions.Count == 0) return;

        // --- AABB from SharpDX Vector3 positions ---
        var min = new Vector3(float.MaxValue, float.MaxValue, float.MaxValue);
        var max = new Vector3(float.MinValue, float.MinValue, float.MinValue);

        foreach (var p in geo.Positions)
        {
            min = Vector3.Minimize(min, p);
            max = Vector3.Maximize(max, p);
        }

        var center = (min + max) * 0.5f;
        var size = max - min;

        float radius = Math.Max(size.X, Math.Max(size.Y, size.Z)) * 0.5f;
        if (radius < 1e-6f) radius = 10f;
        radius *= padding;

        // --- must be a SharpDX PerspectiveCamera in HelixToolkit.Wpf.SharpDX ---
        if (viewport.Camera is not PerspectiveCamera cam)
        {
            // fallback
            viewport.ZoomExtents();
            return;
        }

        if (cam.FieldOfView < 5) cam.FieldOfView = 45;

        float fovRad = (float)(cam.FieldOfView * Math.PI / 180.0);
        float dist = radius / (float)Math.Tan(fovRad * 0.5f);

        // Use current look direction if valid, else default towards -Z
        var lookDir = cam.LookDirection;
        if (lookDir.LengthSquared() < 1e-6f)
            lookDir = new Vector3(0, 0, -1);

        // IMPORTANT: SharpDX Vector3.Normalize has BOTH instance + static forms.
        // Use static form correctly:
        lookDir = Vector3.Normalize(lookDir);

        cam.Position = center - lookDir * dist;
        cam.LookDirection = center - cam.Position;

        // Near/Far planes (critical for "nothing visible")
        cam.NearPlaneDistance = Math.Max(0.001f, dist / 1000f);
        cam.FarPlaneDistance  = dist * 1000f;
    }
}