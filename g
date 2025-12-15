using HelixToolkit.Wpf;
using System.Windows.Media.Media3D;

public static class CameraHelpers
{
    /// <summary>
    /// Rotate the Helix viewport so you see the plane "edge on" (side view).
    /// Camera looks along a direction lying in the plane; Up is the plane normal.
    /// </summary>
    /// <param name="viewport">Helix viewport.</param>
    /// <param name="plane">Fitted plane (uses Centroid and Normal).</param>
    /// <param name="distance">Camera distance from plane centroid.</param>
    /// <param name="useFirstAxis">
    /// If true use one in-plane axis, if false use the orthogonal in-plane axis
    /// (lets you flip which side you view from).
    /// </param>
    public static void LookSideOnToPlane(
        HelixViewport3D viewport,
        PlaneFitResult plane,
        double distance = 50.0,
        bool useFirstAxis = true)
    {
        if (viewport?.Camera is not ProjectionCamera cam)
            return;

        Vector3D n = plane.Normal;
        if (n.LengthSquared < 1e-12)
            return;
        n.Normalize();

        // Pick a "world up" that is not parallel to the normal
        Vector3D worldUp = new Vector3D(0, 0, 1);
        if (Math.Abs(Vector3D.DotProduct(n, worldUp)) > 0.9)
            worldUp = new Vector3D(0, 1, 0);

        // Build an orthonormal basis {u, v, n} where u, v lie in the plane
        // u is in plane, roughly horizontal (n × worldUp)
        Vector3D u = Vector3D.CrossProduct(n, worldUp);
        if (u.LengthSquared < 1e-12)
            u = new Vector3D(1, 0, 0);
        u.Normalize();

        // v is the other in-plane axis (n × u)
        Vector3D v = Vector3D.CrossProduct(n, u);
        if (v.LengthSquared < 1e-12)
            v = new Vector3D(0, 1, 0);
        v.Normalize();

        // Choose which in-plane axis to look along (this is the "side" direction)
        Vector3D sideDir = useFirstAxis ? u : v;

        // We want LookDirection to point FROM camera TO plane centroid.
        // So camera position = centroid - sideDir * distance
        Point3D target = plane.Centroid;
        Point3D position = target - sideDir * distance;

        cam.Position = position;
        cam.LookDirection = target - position; // towards the plane, along sideDir
        cam.UpDirection = n;                   // plane normal is "up" in the view

        // Do NOT call ZoomExtents() here, that would overwrite our orientation.
        viewport.Camera = cam;
    }
}