using System.Collections.Generic;
using System.Windows;
using System.Windows.Media;
using System.Windows.Media.Media3D;

namespace LLT
{
    public static class PlaneVisualHelper
    {
        /// <summary>
        /// Creates a rectangular plane model aligned with the best-fit plane
        /// defined by centroid + normal and the given inlier points.
        /// Uses only WPF 3D types (Point3D, Vector3D, MeshGeometry3D).
        /// </summary>
        public static GeometryModel3D CreatePlaneModel(
            Point3D centroid,
            Vector3D normal,
            IEnumerable<Point3D> inlierPoints,
            double paddingFactor = 1.1,
            Brush? frontBrush = null,
            Brush? backBrush = null)
        {
            // Normalize plane normal
            if (normal.LengthSquared < 1e-12)
                normal = new Vector3D(0, 0, 1);
            normal.Normalize();

            // Build a local (u, v) basis in the plane
            Vector3D u = Vector3D.CrossProduct(normal, new Vector3D(0, 0, 1));
            if (u.LengthSquared < 1e-8)
                u = Vector3D.CrossProduct(normal, new Vector3D(0, 1, 0));
            u.Normalize();

            Vector3D v = Vector3D.CrossProduct(normal, u);
            v.Normalize();

            // Project all inliers into (u, v) coordinates to find bounds
            double minU = double.MaxValue, maxU = double.MinValue;
            double minV = double.MaxValue, maxV = double.MinValue;

            foreach (var p in inlierPoints)
            {
                Vector3D d  = p - centroid;
                double du   = Vector3D.DotProduct(d, u);
                double dv   = Vector3D.DotProduct(d, v);

                if (du < minU) minU = du;
                if (du > maxU) maxU = du;
                if (dv < minV) minV = dv;
                if (dv > maxV) maxV = dv;
            }

            if (double.IsInfinity(minU) || double.IsInfinity(minV))
            {
                // No points – return a tiny default quad to avoid NaN issues
                minU = minV = -5;
                maxU = maxV = 5;
            }

            // Pad bounds a bit
            double spanU = maxU - minU;
            double spanV = maxV - minV;
            double padU  = spanU * (paddingFactor - 1.0) / 2.0;
            double padV  = spanV * (paddingFactor - 1.0) / 2.0;

            minU -= padU; maxU += padU;
            minV -= padV; maxV += padV;

            // Plane corners back in 3D
            Point3D p00 = centroid + u * minU + v * minV;
            Point3D p10 = centroid + u * maxU + v * minV;
            Point3D p11 = centroid + u * maxU + v * maxV;
            Point3D p01 = centroid + u * minU + v * maxV;

            // Build a simple quad mesh
            var mesh = new MeshGeometry3D
            {
                Positions = new Point3DCollection { p00, p10, p11, p01 },
                TriangleIndices = new Int32Collection { 0, 1, 2, 0, 2, 3 },
                Normals = new Vector3DCollection
                {
                    normal, normal, normal, normal
                },
                TextureCoordinates = new PointCollection
                {
                    new Point(0, 1),
                    new Point(1, 1),
                    new Point(1, 0),
                    new Point(0, 0)
                }
            };

            var front = frontBrush ?? new SolidColorBrush(Color.FromArgb(80, 0, 128, 255)); // semi-transparent blue
            var back  = backBrush  ?? front;

            var mat     = new DiffuseMaterial(front);
            var backMat = new DiffuseMaterial(back);

            return new GeometryModel3D
            {
                Geometry     = mesh,
                Material     = mat,
                BackMaterial = backMat
            };
        }
    }
}