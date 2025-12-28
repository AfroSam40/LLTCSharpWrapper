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
            // closest hit (smallest distance)
            var nearestHit = hits.OrderBy(h => h.Distance).First();
            result = nearestHit.PointHit;
            return true;
        }

        // 2) If no visual hit: build a ray from camera through mouse
        Point3D rayOrigin;
        Vector3D rayDirection;
        if (!Viewport3DHelper.Point2DToRay3D(
                viewport.Viewport,
                mousePosition,
                out rayOrigin,
                out rayDirection))
        {
            // If we can't even get a ray, fallback to "nearest point to camera"
            if (pointCloud == null || pointCloud.Count == 0)
            {
                result = new Point3D();
                return false;
            }

            double bestD2 = double.MaxValue;
            Point3D best = new Point3D();
            foreach (var p in pointCloud)
            {
                double d2 = (p - rayOrigin).LengthSquared;
                if (d2 < bestD2)
                {
                    bestD2 = d2;
                    best = p;
                }
            }

            result = best;
            return true;
        }

        if (pointCloud == null || pointCloud.Count == 0)
        {
            result = new Point3D();
            return false;
        }

        // Normalize direction for safety
        rayDirection.Normalize();
        double dirLen2 = rayDirection.LengthSquared;

        // 3) Find nearest point in the cloud to the ray
        bool found = false;
        double bestRayDist2 = double.MaxValue;
        Point3D bestPoint = new Point3D();

        foreach (var p in pointCloud)
        {
            Vector3D w = p - rayOrigin;

            // parameter t along the ray (projection of w onto ray direction)
            double t = Vector3D.DotProduct(w, rayDirection) / dirLen2;

            if (t <= 0)
            {
                // behind the camera, skip it
                continue;
            }

            Point3D proj = rayOrigin + t * rayDirection;
            double d2 = (p - proj).LengthSquared;

            if (d2 < bestRayDist2)
            {
                bestRayDist2 = d2;
                bestPoint = p;
                found = true;
            }
        }

        // 4) If all points were behind camera (or something weird),
        //    fall back to plain nearest in 3D
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