using System;
using System.Collections.Generic;
using System.Linq;
using System.Windows.Media.Media3D;

namespace PointCloudUtils
{
    public static class PointCloudClustering
    {
        /// <summary>
        /// Extracts the largest spatial cluster from a 3D point cloud using a
        /// grid-hashed neighborhood search (much faster than naive O(N^2) DBSCAN).
        /// 
        /// A "cluster" is defined by points that are connected via neighbors
        /// within 'radius'.
        /// </summary>
        /// <param name="points">Input point cloud.</param>
        /// <param name="radius">
        /// Neighbor distance threshold (same units as your coordinates, e.g. mm).
        /// </param>
        /// <param name="minClusterSize">
        /// Ignore clusters smaller than this (treated as noise).
        /// </param>
        /// <returns>
        /// A Point3DCollection containing the largest cluster's points
        /// (or empty if nothing valid).
        /// </returns>
        public static Point3DCollection ExtractLargestCluster(
            Point3DCollection points,
            double radius,
            int minClusterSize = 50)
        {
            var result = new Point3DCollection();
            if (points == null || points.Count == 0 || radius <= 0)
                return result;

            int n = points.Count;
            double cellSize = radius; // grid spacing
            double r2 = radius * radius;

            // 1) Build spatial hash: grid cell -> list of point indices
            var grid = new Dictionary<(int gx, int gy, int gz), List<int>>(n);

            for (int i = 0; i < n; i++)
            {
                var p = points[i];
                int gx = (int)Math.Floor(p.X / cellSize);
                int gy = (int)Math.Floor(p.Y / cellSize);
                int gz = (int)Math.Floor(p.Z / cellSize);
                var key = (gx, gy, gz);

                if (!grid.TryGetValue(key, out var list))
                {
                    list = new List<int>();
                    grid[key] = list;
                }
                list.Add(i);
            }

            // 2) BFS over points, restricted to local cells
            var visited = new bool[n];
            var queue   = new Queue<int>();

            List<int>? bestClusterIndices = null;

            foreach (var cellEntry in grid)
            {
                foreach (int startIdx in cellEntry.Value)
                {
                    if (visited[startIdx])
                        continue;

                    // Start a new cluster from this point
                    var cluster = new List<int>();
                    queue.Clear();

                    visited[startIdx] = true;
                    queue.Enqueue(startIdx);
                    cluster.Add(startIdx);

                    while (queue.Count > 0)
                    {
                        int idx = queue.Dequeue();
                        var p0 = points[idx];

                        // Find grid cell of this point
                        int gx = (int)Math.Floor(p0.X / cellSize);
                        int gy = (int)Math.Floor(p0.Y / cellSize);
                        int gz = (int)Math.Floor(p0.Z / cellSize);

                        // Check this cell and the 26 neighbors
                        for (int dx = -1; dx <= 1; dx++)
                        {
                            for (int dy = -1; dy <= 1; dy++)
                            {
                                for (int dz = -1; dz <= 1; dz++)
                                {
                                    var key = (gx + dx, gy + dy, gz + dz);
                                    if (!grid.TryGetValue(key, out var neighborList))
                                        continue;

                                    foreach (int j in neighborList)
                                    {
                                        if (visited[j]) continue;

                                        var pj = points[j];
                                        double dxp = pj.X - p0.X;
                                        double dyp = pj.Y - p0.Y;
                                        double dzp = pj.Z - p0.Z;
                                        double dist2 = dxp * dxp + dyp * dyp + dzp * dzp;

                                        if (dist2 <= r2)
                                        {
                                            visited[j] = true;
                                            queue.Enqueue(j);
                                            cluster.Add(j);
                                        }
                                    }
                                }
                            }
                        }
                    }

                    // Keep the largest cluster that passes the min size
                    if (cluster.Count >= minClusterSize)
                    {
                        if (bestClusterIndices == null || cluster.Count > bestClusterIndices.Count)
                        {
                            bestClusterIndices = cluster;
                        }
                    }
                }
            }

            if (bestClusterIndices == null || bestClusterIndices.Count == 0)
                return result;

            // 3) Build resulting Point3DCollection
            foreach (int idx in bestClusterIndices)
                result.Add(points[idx]);

            return result;
        }
    }
}