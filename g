using System;
using System.Linq;
using System.Windows.Media.Media3D;
using HelixToolkit.Wpf;

public static class ViewportHelpers
{
    /// <summary>
    /// Rotates the HelixViewport3D camera so you are looking
    /// perpendicular (normal) to the fitted plane – i.e. a "side view"
    /// of that plane.
    /// </summary>
    /// <param name="viewport">Your HelixViewport3D.</param>
    /// <param name="plane">PlaneFitResult containing Normal, Centroid, InlierPoints.</param>
    /// <param name="distanceFactor">
    /// Multiplier for how far the camera is from the plane (based on plane extent).
    /// </param>
    public static void LookPerpendicularToPlane(
        HelixViewport3D viewport,
        PlaneFitResult plane,
        double distanceFactor = 2.0)
    {
        if (viewport == null || plane == null)
            return;

        // We need a ProjectionCamera to manipulate (Perspective or Orthographic)
        var cam = viewport.Camera as ProjectionCamera;
        if (cam == null)
            return;

        // 1. Normalize plane normal
        var n = plane.Normal;
        if (n.LengthSquared < 1e-12)
            return;
        n.Normalize();

        // 2. Estimate a "radius" of the plane from its inlier points
        double radius = 10.0; // default fallback
        if (plane.InlierPoints != null && plane.InlierPoints.Count > 1)
        {
            var c = plane.Centroid;
            radius = plane.InlierPoints
                .Select(p => (p - c).Length)
                .DefaultIfEmpty(10.0)
                .Max();
        }

        double distance = radius * distanceFactor;
        if (distance < 1.0)
            distance = 1.0;

        // 3. Put the camera on the side of the plane along the normal
        //    (flip sign if you want to view from the other side).
        Point3D target = plane.Centroid;
        Point3D position = target + n * distance; // camera in front of plane
        Vector3D lookDir = target - position;     // points toward the plane

        // 4. Choose an UpDirection that is not parallel to the normal.
        //    Start with world Z; if too parallel, fall back to world X.
        Vector3D worldUp = new Vector3D(0, 0, 1);
        if (Math.Abs(Vector3D.DotProduct(worldUp, n)) > 0.9)
            worldUp = new Vector3D(1, 0, 0);

        // Make UpDirection orthogonal to the look direction
        Vector3D right = Vector3D.CrossProduct(lookDir, worldUp);
        if (right.LengthSquared < 1e-12)
            right = new Vector3D(1, 0, 0);
        right.Normalize();

        Vector3D up = Vector3D.CrossProduct(right, lookDir);
        up.Normalize();

        // 5. Apply to camera
        cam.Position     = position;
        cam.LookDirection = lookDir;
        cam.UpDirection   = up;

        // Optional: adjust near/far planes and zoom to fit
        viewport.ZoomExtents();
    }
}