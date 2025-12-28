using System;
using System.Linq;
using System.Windows;
using System.Windows.Media.Media3D;
using HelixToolkit.Wpf;

public static class HitTestHelpers
{
    /// <summary>
    /// Tries to get either a "real" hit from Helix hit testing,
    /// or, if nothing was hit, the nearest point in the given point cloud
    /// along the ray from camera through mousePosition.
    /// 
    /// Returns true if it could return *something*.
    /// </summary>
    public static bool TryHitOrNearestPoint(
        HelixViewport3D viewport,
        Point mousePosition,
        Point3DCollection pointCloud,
        out Point3D result)
    {
        // 1) First try the normal Helix hit test
        var hits = viewport.FindHits(mousePosition);
        if (hits != null && hits.Count > 0)
        {
            var nearestHit = hits.OrderBy(h => h.Distance).First();
            result = nearestHit.PointHit;
            return true;
        }

        // 2) Build a ray from camera through mouse using Helix's helper
        if (pointCloud == null || pointCloud.Count == 0)
        {
            result = new Point3D();
            return false;
        }

        Ray3D ray;

        try
        {
            // This is the correct Helix method:
            // Ray3D Viewport3DHelper.Point2DtoRay3D(Viewport3D viewport, Point p)
            ray = Viewport3DHelper.Point2DtoRay3D(viewport.Viewport, mousePosition);
        }
        catch
        {
            // If something went wrong, just return the point nearest to the camera position
            var camPos = viewport.Camera.Position;
            double bestD2 = double.MaxValue;
            Point3D best = new Point3D();

            foreach (var p in pointCloud)
            {
                double d2 = (p - camPos).LengthSquared;
                if (d2 < bestD2)
                {
                    bestD2 = d2;
                    best = p;
                }
            }

            result = best;
            return true;
        }

        var rayOrigin = ray.Origin;
        var rayDirection = ray.Direction;
        rayDirection.Normalize();
        double dirLen2 = rayDirection.LengthSquared;

        // 3) Find nearest point in the cloud to the ray
        bool found = false;
        double bestRayDist2 = double.MaxValue;
        Point3D bestPoint = new Point3D();

        foreach (var p in pointCloud)
        {
            Vector3D w = p - rayOrigin;
            double t = Vector3D.DotProduct(w, rayDirection) / dirLen2;

            if (t <= 0)
                continue; // behind camera

            Point3D proj = rayOrigin + t * rayDirection;
            double d2 = (p - proj).LengthSquared;

            if (d2 < bestRayDist2)
            {
                bestRayDist2 = d2;
                bestPoint = p;
                found = true;
            }
        }

        // 4) If all points ended up “behind”, fall back to nearest to camera
        if (!found)
        {
            double bestD2 = double.MaxValue;
            foreach (var p in pointCloud)
            {
                double d2 = (p - rayOrigin).LengthSquared;
                if (d2 < bestD2)
                {
                    bestD2 = d2;
                    bestPoint = p;
                }
            }
            found = true;
        }

        result = bestPoint;
        return found;
    }
}