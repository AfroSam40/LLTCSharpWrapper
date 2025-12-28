using System;
using System.Collections.Generic;
using System.Linq;
using System.Windows.Media.Media3D;

namespace LLT
{
    public static class PointCloudProcessing
    {
        /// <summary>
        /// Finds multiple best-fit (roughly horizontal) planes in a point cloud.
        /// 
        /// 1) Sorts points by Z and builds "height surfaces" where
        ///    |Z - currentSurfaceMeanZ| <= bandThickness.
        /// 2) For each height surface, optionally splits it into XY clusters:
        ///    points closer than xyRadius (in XY) and connected via neighbors
        ///    end up in the same surface.
        /// 3) Fits a horizontal plane z = C (normal = (0,0,1)) to each cluster.
        /// </summary>
        /// <param name="points">Input point cloud.</param>
        /// <param name="bandThickness">
        /// Max allowed |ΔZ| from the running mean Z of a surface
        /// for points to still be considered the same surface.
        /// </param>
        /// <param name="xyRadius">
        /// Maximum XY distance between neighboring points within one surface.
        /// If &lt;= 0, XY clustering is disabled (all points in a height band form one surface).
        /// Units are the same as X/Y of the point cloud (e.g. mm).
        /// </param>
        /// <param name="minPointsPerPlane">
        /// Minimum number of points a surface cluster must have to be accepted.
        /// </param>
        public static List<PlaneFitResult> FitHorizontalPlanesByHeight(
            Point3DCollection points,
            double bandThickness,
            double xyRadius = 0.0,
            int minPointsPerPlane = 100)
        {
            var results = new List<PlaneFitResult>();
            if (points == null || points.Count == 0)
                return results;

            // 1. Sort points by Z (ascending)
            var sorted = points.OrderBy(p => p.Z).ToList();

            // 2. Build height surfaces using a running mean in Z
            var currentSurface = new List<Point3D> { sorted[0] };
            double currentMeanZ = sorted[0].Z;

            for (int i = 1; i < sorted.Count; i++)
            {
                var p = sorted[i];
                double dzToMean = Math.Abs(p.Z - currentMeanZ);

                if (dzToMean <= bandThickness)
                {
                    // Still same height surface
                    currentSurface.Add(p);

                    // incremental mean update
                    int n = currentSurface.Count;
                    currentMeanZ += (p.Z - currentMeanZ) / n;
                }
                else
                {
                    // Finish this height surface
                    AddSurfacePlanes(currentSurface, xyRadius, minPointsPerPlane, results);

                    // Start new surface
                    currentSurface = new List<Point3D> { p };
                    currentMeanZ = p.Z;
                }
            }

            // Final surface
            AddSurfacePlanes(currentSurface, xyRadius, minPointsPerPlane, results);

            return results;
        }

        /// <summary>
        /// Takes the accumulated points of one height surface, optionally splits
        /// them into XY clusters, fits a horizontal plane to each, and appends
        /// the results to 'results'.
        /// </summary>
        private static void AddSurfacePlanes(
            List<Point3D> surfacePoints,
            double xyRadius,
            int minPointsPerPlane,
            List<PlaneFitResult> results)
        {
            if (surfacePoints == null || surfacePoints.Count < minPointsPerPlane)
                return;

            IEnumerable<List<Point3D>> clusters;

            if (xyRadius > 0)
                clusters = SplitSurfaceByXY(surfacePoints, xyRadius, minPointsPerPlane);
            else
                clusters = new[] { surfacePoints };

            foreach (var cluster in clusters)
            {
                if (cluster.Count < minPointsPerPlane)
                    continue;

                var plane = FitHorizontalPlane(cluster);
                if (plane != null)
                    results.Add(plane);
            }
        }

        /// <summary>
        /// Splits a set of points (all at similar Z) into XY clusters:
        /// points closer than 'xyRadius' (in XY) and connected via neighbors
        /// end up in the same cluster. Roughly like a fast DBSCAN in 2D.
        /// </summary>
        private static IEnumerable<List<Point3D>> SplitSurfaceByXY(
            List<Point3D> pts,
            double xyRadius,
            int minPointsPerCluster)
        {
            var clusters = new List<List<Point3D>>();
            if (pts == null || pts.Count == 0)
                return clusters;

            double cellSize = xyRadius;
            double r2 = xyRadius * xyRadius;

            // Grid: maps (ix,iy) -> list of indices
            var grid = new Dictionary<(int ix, int iy), List<int>>();

            for (int i = 0; i < pts.Count; i++)
            {
                var p = pts[i];
                int ix = (int)Math.Floor(p.X / cellSize);
                int iy = (int)Math.Floor(p.Y / cellSize);
                var key = (ix, iy);

                if (!grid.TryGetValue(key, out var list))
                {
                    list = new List<int>();
                    grid[key] = list;
                }

                list.Add(i);
            }

            var visited = new bool[pts.Count];

            for (int i = 0; i < pts.Count; i++)
            {
                if (visited[i]) continue;

                var cluster = new List<Point3D>();
                var queue = new Queue<int>();
                queue.Enqueue(i);
                visited[i] = true;

                while (queue.Count > 0)
                {
                    int idx = queue.Dequeue();
                    var p = pts[idx];
                    cluster.Add(p);

                    int ix = (int)Math.Floor(p.X / cellSize);
                    int iy = (int)Math.Floor(p.Y / cellSize);

                    // Check this cell and neighbors
                    for (int dx = -1; dx <= 1; dx++)
                    {
                        for (int dy = -1; dy <= 1; dy++)
                        {
                            var key = (ix + dx, iy + dy);
                            if (!grid.TryGetValue(key, out var neighIdxs))
                                continue;

                            foreach (int j in neighIdxs)
                            {
                                if (visited[j]) continue;

                                var q = pts[j];
                                double dxp = q.X - p.X;
                                double dyp = q.Y - p.Y;

                                if (dxp * dxp + dyp * dyp <= r2)
                                {
                                    visited[j] = true;
                                    queue.Enqueue(j);
                                }
                            }
                        }
                    }
                }

                if (cluster.Count >= minPointsPerCluster)
                    clusters.Add(cluster);
            }

            return clusters;
        }

        /// <summary>
        /// Fits a purely horizontal plane z = C
        /// (A = 0, B = 0, Normal = (0,0,1)) to the given points.
        /// Fills AverageError and Rmse based on vertical residuals (Z - C).
        /// </summary>
        private static PlaneFitResult? FitHorizontalPlane(List<Point3D> pts)
        {
            int n = pts.Count;
            if (n < 3)
                return null;

            double sumX = 0, sumY = 0, sumZ = 0;
            foreach (var p in pts)
            {
                sumX += p.X;
                sumY += p.Y;
                sumZ += p.Z;
            }

            double cx = sumX / n;
            double cy = sumY / n;
            double cz = sumZ / n;

            double A = 0.0;
            double B = 0.0;
            double C = cz;

            var normal = new Vector3D(0, 0, 1);

            double absErrSum = 0;
            double sqErrSum  = 0;
            foreach (var p in pts)
            {
                double res = p.Z - C;
                absErrSum += Math.Abs(res);
                sqErrSum  += res * res;
            }

            double avgErr = absErrSum / n;
            double rmse   = Math.Sqrt(sqErrSum / n);

            var centroid = new Point3D(cx, cy, cz);

            return new PlaneFitResult
            {
                A = A,
                B = B,
                C = C,
                Normal = normal,
                Centroid = centroid,
                Origin = centroid,   // reuse centroid as origin for this plane
                AverageError = avgErr,
                Rms = rmse,
                InlierPoints = new List<Point3D>(pts)
            };
        }
    }
}