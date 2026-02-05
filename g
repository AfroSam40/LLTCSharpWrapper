using System;
using System.Collections.Generic;
using System.Linq;
using System.Windows;
using HelixToolkit.Wpf.SharpDX;
using SharpDX;

public static class Viewport3DXPicking
{
    /// <summary>
    /// Returns the nearest point in a point cloud to the clicked location in a Viewport3DX.
    /// 1) If Helix hit-test hits something, returns the closest hit point.
    /// 2) Otherwise casts a ray through the mouse pixel and returns the cloud point nearest that ray.
    /// Always tries to return something if points has at least 1 point.
    /// </summary>
    public static bool TryHitOrNearestPoint(
        Viewport3DX viewport,
        Point mousePosition,
        IList<Vector3> points,
        out Vector3 nearestPoint)
    {
        nearestPoint = default;

        if (viewport == null) return false;
        if (points == null || points.Count == 0) return false;

        // 1) Try Helix hit test first (if you clicked a renderable that supports hit testing)
        //    In SharpDX Helix, this often works for meshes; point clouds depend on how they are rendered.
        var hits = viewport.FindHits(mousePosition);
        if (hits != null && hits.Count > 0)
        {
            var best = hits.OrderBy(h => h.Distance).FirstOrDefault();
            if (best != null)
            {
                // PointHit is a SharpDX.Vector3 in SharpDX Helix hit results
                nearestPoint = best.PointHit;
                return true;
            }
        }

        // 2) No hit: build a world-space ray from camera through mouse pixel, then find nearest point to the ray.
        if (!TryBuildPickRay(viewport, mousePosition, out var rayOrigin, out var rayDir))
        {
            // Fallback: nearest to camera position (still "always returns something")
            var camPos = GetCameraPosition(viewport);
            float bestD2 = float.MaxValue;
            Vector3 bestP = points[0];

            for (int i = 0; i < points.Count; i++)
            {
                var d2 = Vector3.DistanceSquared(points[i], camPos);
                if (d2 < bestD2) { bestD2 = d2; bestP = points[i]; }
            }

            nearestPoint = bestP;
            return true;
        }

        // Normalize direction for safety
        rayDir.Normalize();

        // Scan for nearest point to ray
        bool found = false;
        float bestRayDist2 = float.MaxValue;
        Vector3 bestPoint = points[0];

        for (int i = 0; i < points.Count; i++)
        {
            var p = points[i];
            var w = p - rayOrigin;

            float t = Vector3.Dot(w, rayDir); // since rayDir is normalized
            if (t <= 0) continue; // behind camera

            var proj = rayOrigin + rayDir * t;
            float d2 = Vector3.DistanceSquared(p, proj);

            if (d2 < bestRayDist2)
            {
                bestRayDist2 = d2;
                bestPoint = p;
                found = true;
            }
        }

        if (!found)
        {
            // Fallback: nearest to ray origin (camera)
            float bestD2 = float.MaxValue;
            for (int i = 0; i < points.Count; i++)
            {
                float d2 = Vector3.DistanceSquared(points[i], rayOrigin);
                if (d2 < bestD2) { bestD2 = d2; bestPoint = points[i]; }
            }
        }

        nearestPoint = bestPoint;
        return true;
    }

    /// <summary>
    /// Builds a pick ray in world space by unprojecting mouse position at near/far depths.
    /// Uses camera view/projection matrices.
    /// </summary>
    private static bool TryBuildPickRay(
        Viewport3DX viewport,
        Point mouse,
        out Vector3 origin,
        out Vector3 direction)
    {
        origin = default;
        direction = default;

        float w = (float)viewport.ActualWidth;
        float h = (float)viewport.ActualHeight;
        if (w <= 1 || h <= 1) return false;

        // NDC coords in [-1,1]
        float ndcX = (float)(2.0 * mouse.X / w - 1.0);
        float ndcY = (float)(1.0 - 2.0 * mouse.Y / h);

        // Need view + projection
        var cam = viewport.Camera;
        if (cam == null) return false;

        // HelixToolkit.Wpf.SharpDX cameras provide these matrix creators
        var view = cam.CreateViewMatrix();
        var proj = cam.CreateProjectionMatrix((float)(w / h));
        var viewProj = view * proj;

        Matrix invViewProj;
        if (!Matrix.Invert(viewProj, out invViewProj))
            return false;

        // Unproject near and far points (z=0 near, z=1 far in NDC)
        var near4 = Vector3.TransformCoordinate(new Vector3(ndcX, ndcY, 0f), invViewProj);
        var far4  = Vector3.TransformCoordinate(new Vector3(ndcX, ndcY, 1f), invViewProj);

        origin = near4;
        direction = far4 - near4;

        if (direction.LengthSquared() < 1e-12f) return false;
        direction.Normalize();
        return true;
    }

    private static Vector3 GetCameraPosition(Viewport3DX viewport)
    {
        // Most Helix SharpDX cameras expose Position (PerspectiveCamera/OrthographicCamera).
        // If not, we fallback to origin.
        if (viewport.Camera is ProjectionCamera pc)
            return new Vector3((float)pc.Position.X, (float)pc.Position.Y, (float)pc.Position.Z);

        return Vector3.Zero;
    }
}