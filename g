using System;
using System.Collections.Generic;
using System.Linq;
using System.Windows;                 // System.Windows.Point
using HelixToolkit.Wpf.SharpDX;      // Viewport3DX
using SharpDX;                       // SharpDX.Vector3, SharpDX.Matrix

public static class Viewport3DXPicking
{
    public static bool TryHitOrNearestPoint(
        Viewport3DX viewport,
        System.Windows.Point mousePosition,
        IList<SharpDX.Vector3> points,
        out SharpDX.Vector3 nearestPoint)
    {
        nearestPoint = default;

        if (viewport == null) return false;
        if (points == null || points.Count == 0) return false;

        // 1) Try Helix hit test first
        var hits = viewport.FindHits(mousePosition);
        if (hits != null && hits.Count > 0)
        {
            var best = hits.OrderBy(h => h.Distance).FirstOrDefault();
            if (best != null)
            {
                nearestPoint = best.PointHit; // SharpDX.Vector3
                return true;
            }
        }

        // 2) No hit: build a ray via UnProject at near/far
        if (!TryBuildPickRay(viewport, mousePosition, out var rayOrigin, out var rayDir))
        {
            // fallback: nearest to camera
            var camPos = GetCameraPosition(viewport);
            float bestD2 = float.MaxValue;
            var bestP = points[0];

            for (int i = 0; i < points.Count; i++)
            {
                float d2 = SharpDX.Vector3.DistanceSquared(points[i], camPos);
                if (d2 < bestD2) { bestD2 = d2; bestP = points[i]; }
            }

            nearestPoint = bestP;
            return true;
        }

        rayDir = SharpDX.Vector3.Normalize(rayDir);

        // 3) Find nearest point to ray (O(N))
        bool found = false;
        float bestRayD2 = float.MaxValue;
        var bestPoint = points[0];

        for (int i = 0; i < points.Count; i++)
        {
            var p = points[i];
            var w = p - rayOrigin;

            float t = SharpDX.Vector3.Dot(w, rayDir);  // rayDir normalized
            if (t <= 0) continue;

            var proj = rayOrigin + rayDir * t;
            float d2 = SharpDX.Vector3.DistanceSquared(p, proj);

            if (d2 < bestRayD2)
            {
                bestRayD2 = d2;
                bestPoint = p;
                found = true;
            }
        }

        if (!found)
        {
            // fallback: nearest to origin
            float bestD2 = float.MaxValue;
            for (int i = 0; i < points.Count; i++)
            {
                float d2 = SharpDX.Vector3.DistanceSquared(points[i], rayOrigin);
                if (d2 < bestD2) { bestD2 = d2; bestPoint = points[i]; }
            }
        }

        nearestPoint = bestPoint;
        return true;
    }

    private static bool TryBuildPickRay(
        Viewport3DX viewport,
        System.Windows.Point mouse,
        out SharpDX.Vector3 origin,
        out SharpDX.Vector3 direction)
    {
        origin = default;
        direction = default;

        float w = (float)viewport.ActualWidth;
        float h = (float)viewport.ActualHeight;
        if (w <= 1 || h <= 1) return false;

        var cam = viewport.Camera;
        if (cam == null) return false;

        // IMPORTANT: Helix SharpDX uses its own camera matrices via viewport
        // These are available from viewport.RenderHost
        var renderHost = viewport.RenderHost;
        if (renderHost == null) return false;

        var view = renderHost.RenderContext.ViewMatrix;
        var proj = renderHost.RenderContext.ProjectionMatrix;
        var viewProj = view * proj;

        SharpDX.Matrix invViewProj;
        if (!SharpDX.Matrix.Invert(viewProj, out invViewProj))
            return false;

        // NDC in [-1,1]
        float ndcX = (float)(2.0 * mouse.X / w - 1.0);
        float ndcY = (float)(1.0 - 2.0 * mouse.Y / h);

        // unproject z=0 (near), z=1 (far)
        var near = SharpDX.Vector3.TransformCoordinate(new SharpDX.Vector3(ndcX, ndcY, 0f), invViewProj);
        var far  = SharpDX.Vector3.TransformCoordinate(new SharpDX.Vector3(ndcX, ndcY, 1f), invViewProj);

        origin = near;
        direction = far - near;

        return direction.LengthSquared() > 1e-12f;
    }

    private static SharpDX.Vector3 GetCameraPosition(Viewport3DX viewport)
    {
        var cam = viewport.Camera;
        if (cam == null) return SharpDX.Vector3.Zero;

        // HelixToolkit.Wpf.SharpDX cameras expose Position as SharpDX.Vector3
        return cam.Position;
    }
}