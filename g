using System;
using System.Collections.Generic;
using System.Linq;
using System.Windows.Media.Media3D;

namespace PointCloudUtils
{
    public static class PointCloudPlaneFitting
    {
        /// <summary>
        /// PCA/SVD-style best-fit plane in full 3D.
        /// Finds the plane that minimizes the orthogonal distance
        /// from all points to the plane (true least-squares plane).
        ///
        /// Returns null if there are fewer than 3 points or the
        /// covariance is degenerate.
        /// </summary>
        public static PlaneFitResult? FitPlanePca3D(Point3DCollection points)
        {
            if (points == null || points.Count < 3)
                return null;

            int n = points.Count;

            // 1) Centroid
            double cx = 0, cy = 0, cz = 0;
            foreach (var p in points)
            {
                cx += p.X;
                cy += p.Y;
                cz += p.Z;
            }
            cx /= n;
            cy /= n;
            cz /= n;

            // 2) Covariance matrix of centered points
            double cxx = 0, cxy = 0, cxz = 0;
            double cyy = 0, cyz = 0, czz = 0;

            foreach (var p in points)
            {
                double dx = p.X - cx;
                double dy = p.Y - cy;
                double dz = p.Z - cz;

                cxx += dx * dx;
                cxy += dx * dy;
                cxz += dx * dz;
                cyy += dy * dy;
                cyz += dy * dz;
                czz += dz * dz;
            }

            // Normalize by n (optional; doesn't change eigenvectors)
            cxx /= n; cxy /= n; cxz /= n;
            cyy /= n; cyz /= n; czz /= n;

            // Symmetric covariance matrix C:
            // [ cxx  cxy  cxz ]
            // [ cxy  cyy  cyz ]
            // [ cxz  cyz  czz ]

            // 3) We want eigenvector with smallest eigenvalue.
            // Use power iteration on M = (trace(C) * I - C) to get
            // the largest eigenvalue of M, which corresponds to the
            // smallest eigenvalue of C.

            double trace = cxx + cyy + czz;
            if (Math.Abs(trace) < 1e-15)
                return null; // all points identical or extremely degenerate

            // Build M = trace*I - C
            double m00 = trace - cxx;
            double m01 = -cxy;
            double m02 = -cxz;

            double m10 = -cxy;
            double m11 = trace - cyy;
            double m12 = -cyz;

            double m20 = -cxz;
            double m21 = -cyz;
            double m22 = trace - czz;

            // Initial guess for normal (slightly biased towards Z)
            double nx = 0.0, ny = 0.0, nz = 1.0;

            for (int iter = 0; iter < 32; iter++)
            {
                // v' = M * v
                double xNew = m00 * nx + m01 * ny + m02 * nz;
                double yNew = m10 * nx + m11 * ny + m12 * nz;
                double zNew = m20 * nx + m21 * ny + m22 * nz;

                double len = Math.Sqrt(xNew * xNew + yNew * yNew + zNew * zNew);
                if (len < 1e-15)
                    break;

                nx = xNew / len;
                ny = yNew / len;
                nz = zNew / len;
            }

            // Normal vector (ensure it's normalized)
            var normal = new Vector3D(nx, ny, nz);
            if (normal.Length < 1e-15)
                return null;
            normal.Normalize();

            // 4) Plane in point-normal form:
            //    (p - c) · n = 0   =>   n·p + d = 0, where d = -n·c
            var centroid = new Point3D(cx, cy, cz);
            double d = -(normal.X * cx + normal.Y * cy + normal.Z * cz);

            // For compatibility with your existing code that uses
            // z = A*x + B*y + C, solve for A,B,C if n.Z is not too small.
            double A = 0, B = 0, C = 0;
            if (Math.Abs(normal.Z) > 1e-8)
            {
                // From n_x x + n_y y + n_z z + d = 0 and z = A x + B y + C:
                // A = -n_x / n_z, B = -n_y / n_z, C = -d / n_z
                A = -normal.X / normal.Z;
                B = -normal.Y / normal.Z;
                C = -d / normal.Z;
            }
            else
            {
                // Plane is nearly vertical; z = A x + B y + C is ill-defined.
                // Keep A,B,C = 0; you'll still have Normal + Centroid.
            }

            // 5) Compute average orthogonal distance (flatness-ish metric)
            double errSum = 0;
            foreach (var p in points)
            {
                // signed distance = ( (p - c) · n )
                double dist = ( (p.X - cx) * normal.X
                              + (p.Y - cy) * normal.Y
                              + (p.Z - cz) * normal.Z );
                errSum += Math.Abs(dist);
            }
            double avgErr = errSum / n;

            return new PlaneFitResult
            {
                A = A,
                B = B,
                C = C,
                Normal = normal,
                Centroid = centroid,
                AverageError = avgErr,
                InlierPoints = points.ToList()
            };
        }
    }
}