using System;
using System.Windows.Media.Media3D;

public static class PlaneMath
{
    /// <summary>
    /// Builds a PlaneFitResult for the plane that is perpendicular to <paramref name="basePlane"/> and passes through <paramref name="p"/>.
    /// Returned plane is expressed in your PlaneFitResult form (A,B,C): z = A*x + B*y + C,
    /// plus Normal + Centroid. (InlierPoints/Errors are not meaningful here and left empty/0.)
    /// </summary>
    public static PlaneFitResult PerpendicularPlaneAtPointAsFitResult(PlaneFitResult basePlane, Point3D p)
    {
        var n = basePlane.Normal;
        if (n.LengthSquared < 1e-18) throw new ArgumentException("Base plane normal is zero.", nameof(basePlane));
        n.Normalize();

        // pick a stable direction in the base plane
        var axis = (Math.Abs(n.Z) < 0.9) ? new Vector3D(0, 0, 1) : new Vector3D(0, 1, 0);
        var u = Vector3D.CrossProduct(n, axis);
        if (u.LengthSquared < 1e-18) { axis = new Vector3D(1, 0, 0); u = Vector3D.CrossProduct(n, axis); }
        u.Normalize();

        // normal of the perpendicular plane (contains n and passes through p)
        var m = Vector3D.CrossProduct(n, u);
        if (m.LengthSquared < 1e-18) throw new InvalidOperationException("Failed to form a perpendicular plane normal.");
        m.Normalize();

        // Plane: m·X + D = 0, passing through p => D = -m·p
        double D = -(m.X * p.X + m.Y * p.Y + m.Z * p.Z);

        // Convert to z = A*x + B*y + C (only valid if m.Z != 0)
        if (Math.Abs(m.Z) < 1e-9)
        {
            // Vertical plane: cannot be represented as z = A x + B y + C.
            // If your SharpDX plane patch builder requires PlaneFitResult(A,B,C),
            // it won't be able to draw true vertical planes without changing it to use (Normal,D).
            throw new InvalidOperationException(
                "Perpendicular plane is (near) vertical (Normal.Z ~ 0) and cannot be represented as z = A*x + B*y + C.");
        }

        // mX*x + mY*y + mZ*z + D = 0 => z = -(mX/mZ)x - (mY/mZ)y - D/mZ
        double A = -(m.X / m.Z);
        double B = -(m.Y / m.Z);
        double C = -(D / m.Z);

        return new PlaneFitResult
        {
            A = A,
            B = B,
            C = C,
            Normal = m,     // keep the true normal
            Centroid = p,   // plane passes through p
            AverageError = 0,
            InlierPoints = new System.Collections.Generic.List<Point3D>()
        };
    }
}
```0