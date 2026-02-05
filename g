using HelixToolkit.Wpf.SharpDX;
using SharpDX;

// Call this after setting model.Geometry (or after you center it)
public static void FixClippingAndZoom(HelixViewport3DX vp, PointGeometry3D geo)
{
    if (vp?.Camera is not ProjectionCamera cam || geo?.Positions == null || geo.Positions.Count == 0)
        return;

    // Compute bounds
    var b = geo.Positions.BoundingBox;
    var center = (b.Minimum + b.Maximum) * 0.5f;
    var diag = (b.Maximum - b.Minimum).Length();
    if (diag <= 1e-6f) diag = 1f;

    // Set clip planes wide enough (key part)
    cam.NearPlaneDistance = Math.Max(1e-3, diag * 0.001);  // small but > 0
    cam.FarPlaneDistance  = diag * 100.0;                  // big

    vp.ZoomExtents();
}