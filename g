using System.Windows.Media.Media3D;

namespace LLT
{
    public static class PointCloudProcessing
    {
        /// <summary>
        /// Removes all points that lie below the given plane (z = A*x + B*y + C).
        /// </summary>
        /// <param name="points">Input point cloud.</param>
        /// <param name="plane">Plane fit result (A,B,C, etc.).</param>
        /// <param name="margin">
        /// Optional margin in Z. 
        /// Points with Z &lt; (zPlane - margin) are removed.
        /// Use a small positive value to be tolerant of noise.
        /// </param>
        /// <returns>A new Point3DCollection containing only points on or above the plane.</returns>
        public static Point3DCollection RemovePointsBelowPlane(
            Point3DCollection points,
            PlaneFitResult plane,
            double margin = 0.0)
        {
            if (points == null || points.Count == 0)
                return new Point3DCollection();

            var result = new Point3DCollection(points.Count);

            foreach (var p in points)
            {
                // Plane height at (x,y)
                double zPlane = plane.A * p.X + plane.B * p.Y + plane.C;

                // Keep points whose Z is on or above the plane (within margin)
                if (p.Z >= zPlane - margin)
                {
                    result.Add(p);
                }
            }

            return result;
        }
    }
}