using System;
using System.Collections.Generic;
using System.Windows.Media.Media3D;

public static class PlaneHelpers
{
    /// <summary>
    /// Builds a PlaneFitResult representing a plane perpendicular to basePlane at atPoint,
    /// and populates InlierPoints with an artificial patch (grid) so existing patch builders
    /// that rely on InlierPoints can render it.
    /// </summary>
    /// <param name="basePlane">Existing fitted plane.</param>
    /// <param name="atPoint">Point where the perpendicular plane should pass through.</param>
    /// <param name="halfSize">Half-width of the patch in your units (e.g. mm). Patch spans 2*halfSize.</param>
    /// <param name="grid">Grid resolution per side. 9 => 9x9 points. Keep modest for speed.</param>
    public static PlaneFitResult PerpendicularPlaneAtPointAsFitResult(
        PlaneFitResult basePlane,
        Point3D atPoint,
        double halfSize = 25.0,
        int grid = 11)
    {
        if (basePlane == null) throw new ArgumentNullException(nameof(basePlane));
        if (grid < 2) grid = 2;

        // Base normal (unit)
        Vector3D n0 = basePlane.Normal;
        if (n0.LengthSquared < 1e-12) throw new ArgumentException("basePlane.Normal is invalid.");
        n0.Normalize();

        // Choose a direction lying in the base plane to define the perpendicular plane normal.
        // Project an arbitrary axis onto base plane to get a stable "in-plane" direction u.
        Vector3D axis = Math.Abs(n0.Z) < 0.9 ? new Vector3D(0, 0, 1) : new Vector3D(0, 1, 0);
        Vector3D u = axis - Vector3D.DotProduct(axis, n0) * n0; // remove normal component
        if (u.LengthSquared < 1e-12)
            u = new Vector3D(1, 0, 0) - Vector3D.DotProduct(new Vector3D(1, 0, 0), n0) * n0;
        u.Normalize();

        // Perpendicular plane normal: lies in base plane (u) and thus is perpendicular to base normal
        Vector3D n1 = u; // normal of the perpendicular plane

        // Build an orthonormal basis on the perpendicular plane:
        // v1 is along base normal (in the perpendicular plane), and w1 is orthogonal direction in plane.
        Vector3D v1 = n0;                 // in-plane direction #1
        Vector3D w1 = Vector3D.CrossProduct(n1, v1);  // in-plane direction #2
        if (w1.LengthSquared < 1e-12)
            w1 = Vector3D.CrossProduct(n1, new Vector3D(0, 0, 1));
        w1.Normalize();
        v1 = Vector3D.CrossProduct(w1, n1); // re-orthogonalize
        v1.Normalize();

        // Create a grid of "inlier" points on the plane centered at atPoint
        var pts = new List<Point3D>(grid * grid);
        double step = (2.0 * halfSize) / (grid - 1);

        // Center the patch on atPoint
        for (int iy = 0; iy < grid; iy++)
        {
            double ty = -halfSize + iy * step;
            for (int ix = 0; ix < grid; ix++)
            {
                double tx = -halfSize + ix * step;
                // atPoint + tx * v1 + ty * w1 lies on the perpendicular plane
                pts.Add(atPoint + tx * v1 + ty * w1);
            }
        }

        // Return as PlaneFitResult (A/B/C are not meaningful for arbitrary orientation; keep 0)
        return new PlaneFitResult
        {
            A = 0,
            B = 0,
            C = 0,
            Normal = n1,          // perpendicular plane normal
            Centroid = atPoint,   // patch centered here
            AverageError = 0,
            InlierPoints = pts
        };
    }
}