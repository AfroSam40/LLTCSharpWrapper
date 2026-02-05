using SharpDX;
using HelixToolkit.Wpf.SharpDX;

public static void FramePointsInView(Viewport3DX viewport, PointGeometry3D geo, float padding = 1.2f)
{
    if (viewport?.Camera == null) return;
    if (geo?.Positions == null || geo.Positions.Count == 0) return;

    // Compute AABB
    var min = new Vector3(float.MaxValue);
    var max = new Vector3(float.MinValue);

    foreach (var p in geo.Positions)
    {
        min = Vector3.Min(min, p);
        max = Vector3.Max(max, p);
    }

    var center = (min + max) * 0.5f;
    var size = max - min;
    var radius = Math.Max(size.X, Math.Max(size.Y, size.Z)) * 0.5f;
    if (radius <= 1e-6f) radius = 10f; // degenerate fallback

    radius *= padding;

    // Perspective camera: place camera along its LookDirection opposite
    var cam = viewport.Camera as PerspectiveCamera;
    if (cam == null)
    {
        // If you’re not actually using PerspectiveCamera, still try ZoomExtents
        viewport.ZoomExtents(0);
        return;
    }

    // Make sure FOV is sane
    if (cam.FieldOfView < 5) cam.FieldOfView = 45;

    // Distance so object fits: d = r / tan(fov/2)
    float fovRad = (float)(cam.FieldOfView * Math.PI / 180.0);
    float dist = radius / (float)Math.Tan(fovRad * 0.5f);

    // Use camera's current look direction if valid, else default -Z
    var lookDir = cam.LookDirection;
    if (lookDir.LengthSquared() < 1e-6f)
        lookDir = new Vector3(0, 0, -1);

    lookDir = Vector3.Normalize(lookDir);

    cam.Position = center - lookDir * dist;
    cam.LookDirection = center - cam.Position;

    // These matter a LOT for “nothing visible”
    cam.NearPlaneDistance = Math.Max(0.001, dist / 1000.0);
    cam.FarPlaneDistance  = dist * 1000.0;
}