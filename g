using System;
using System.Collections.Generic;
using System.Linq;
using System.Windows.Media.Media3D;

namespace PointCloudUtils
{
    /// <summary>
    /// Result of fitting a (roughly horizontal) plane.
    /// Plane equation: z = A*x + B*y + C when Normal.Z is not ~0.
    /// </summary>
    public class PlaneFitResult
    {
        public double A { get; set; }       // z = A*x + B*y + C
        public double B { get; set; }
        public double C { get; set; }

        /// <summary>Plane normal (unit vector).</summary>
        public Vector3D Normal { get; set; }

        /// <summary>Centroid of inlier points.</summary>
        public Point3D Centroid { get; set; }

        /// <summary>Average orthogonal distance of inlier points to the plane.</summary>
        public double AverageError { get; set; }

        /// <summary>Points used to fit this plane.</summary>
        public List<Point3D> InlierPoints { get; set; } = new List<Point3D>();
    }

    public static class PointCloudPlaneFitting
    {
        /// <summary>
        /// Find multiple (roughly horizontal) planes in a point cloud by:
        /// 1) Sorting by Z
        /// 2) Grouping into Z-bands of thickness 'bandThickness'
        /// 3) Inside each band, clustering points in XY using 'xyRadius'
        /// 4) Fitting a PCA-based 3D plane to each cluster with &gt;= minPointsPerPlane points.
        /// 
        /// Returns one PlaneFitResult per detected surface patch.
        /// </summary>
        public static List<PlaneFitResult> FitHorizontalPlanesByHeightAndXY(
            Point3DCollection points,
            double bandThickness,
            double xyRadius,
            int minPointsPerPlane = 100)
        {
            var results = new List<PlaneFitResult>();
            if (points == null || points.Count == 0)
                return results;

            if (bandThickness <= 0)
                throw new ArgumentOutOfRangeException(nameof(bandThickness));
            if (xyRadius <= 0)
                throw new ArgumentOutOfRangeException(nameof(xyRadius));

            double xyRadius2 = xyRadius * xyRadius;

            // 1) Sort by Z
            var sorted = points.OrderBy(p => p.Z).ToList();
            int index = 0;

            while (index < sorted.Count)
            {
                // Start a new Z-band
                double bandStartZ = sorted[index].Z;
                var bandPoints = new List<Point3D>();

                // Collect all points whose Z is within bandThickness of bandStartZ
                int j = index;
                while (j < sorted.Count &&
                       Math.Abs(sorted[j].Z - bandStartZ) <= bandThickness)
                {
                    bandPoints.Add(sorted[j]);
                    j++;
                }

                // 2) Within this band, cluster by XY proximity
                int m = bandPoints.Count;
                if (m >= minPointsPerPlane)
                {
                    var visited = new bool[m];

                    for (int i = 0; i < m; i++)
                    {
                        if (visited[i])
                            continue;

                        // Region growing in XY
                        var cluster = new List<Point3D>();
                        var stack = new Stack<int>();
                        stack.Push(i);
                        visited[i] = true;

                        while (stack.Count > 0)
                        {
                            int idx = stack.Pop();
                            var p = bandPoints[idx];
                            cluster.Add(p);

                            // Brute-force neighbor search (fine for medium point counts;
                            // optimize with spatial index if needed)
                            for (int k = 0; k < m; k++)
                            {
                                if (visited[k]) continue;
                                var q = bandPoints[k];
                                double dx = q.X - p.X;
                                double dy = q.Y - p.Y;
                                if (dx * dx + dy * dy <= xyRadius2)
                                {
                                    visited[k] = true;
                                    stack.Push(k);
                                }
                            }
                        }

                        if (cluster.Count >= minPointsPerPlane)
                        {
                            var plane = FitPlanePca3D(cluster);
                            if (plane != null)
                                results.Add(plane);
                        }
                    }
                }

                // Move to next Z-band
                index = j;
            }

            return results;
        }

        /// <summary>
        /// PCA-style best-fit plane to a set of 3D points.
        /// Finds plane minimizing orthogonal distances (true least-squares).
        /// </summary>
        private static PlaneFitResult? FitPlanePca3D(List<Point3D> pts)
        {
            if (pts == null || pts.Count < 3)
                return null;

            int n = pts.Count;

            // 1) Centroid
            double cx = 0, cy = 0, cz = 0;
            foreach (var p in pts)
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

            foreach (var p in pts)
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

            cxx /= n; cxy /= n; cxz /= n;
            cyy /= n; cyz /= n; czz /= n;

            double trace = cxx + cyy + czz;
            if (Math.Abs(trace) < 1e-15)
                return null;

            // M = trace*I - C  (largest eigenvector of M = smallest of C)
            double m00 = trace - cxx;
            double m01 = -cxy;
            double m02 = -cxz;
            double m10 = -cxy;
            double m11 = trace - cyy;
            double m12 = -cyz;
            double m20 = -cxz;
            double m21 = -cyz;
            double m22 = trace - czz;

            // Power iteration
            double nx = 0, ny = 0, nz = 1; // bias towards Z
            for (int iter = 0; iter < 32; iter++)
            {
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

            var normal = new Vector3D(nx, ny, nz);
            if (normal.Length < 1e-15)
                return null;
            normal.Normalize();

            var centroid = new Point3D(cx, cy, cz);

            // Convert to z = A x + B y + C where possible
            double d = -(normal.X * cx + normal.Y * cy + normal.Z * cz);
            double A = 0, B = 0, C = 0;

            if (Math.Abs(normal.Z) > 1e-8)
            {
                A = -normal.X / normal.Z;
                B = -normal.Y / normal.Z;
                C = -d / normal.Z;
            }

            // Average orthogonal distance (flatness-ish)
            double errSum = 0;
            foreach (var p in pts)
            {
                double dist = ((p.X - cx) * normal.X
                             + (p.Y - cy) * normal.Y
                             + (p.Z - cz) * normal.Z);
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
                InlierPoints = new List<Point3D>(pts)
            };
        }
    }
}