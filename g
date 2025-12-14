using System;
using System.Collections.Generic;
using System.Windows.Media.Media3D;

namespace LLT
{
    public static class PointCloudDiff
    {
        /// <summary>
        /// Computes the "added" object point cloud: points that exist in
        /// <paramref name="after"/> but not in <paramref name="before"/>.
        /// 
        /// Assumes both clouds are in the same coordinate system and aligned.
        /// A point in "after" is considered background if there is at least one
        /// point in "before" within <paramref name="matchRadius"/> distance.
        /// All others are treated as part of the new object.
        /// </summary>
        /// <param name="before">Point cloud before placing the object.</param>
        /// <param name="after">Point cloud after placing the object.</param>
        /// <param name="matchRadius">
        /// Distance tolerance (same units as your points, e.g. mm).
        /// Typical: 0.02–0.10 depending on noise.
        /// </param>
        /// <returns>Point3DCollection representing the isolated new object.</returns>
        public static Point3DCollection ExtractAddedObject(
            Point3DCollection before,
            Point3DCollection after,
            double matchRadius)
        {
            if (before == null) throw new ArgumentNullException(nameof(before));
            if (after == null) throw new ArgumentNullException(nameof(after));
            if (matchRadius <= 0) throw new ArgumentOutOfRangeException(nameof(matchRadius));

            // Use voxel size about the radius (slightly smaller to be safe)
            double cellSize = matchRadius;
            double radiusSq = matchRadius * matchRadius;

            // Spatial hash: (ix, iy, iz) -> list of background points in that cell
            var grid = new Dictionary<(int ix, int iy, int iz), List<Point3D>>();

            foreach (var p in before)
            {
                var key = ToCell(p, cellSize);
                if (!grid.TryGetValue(key, out var list))
                {
                    list = new List<Point3D>();
                    grid[key] = list;
                }
                list.Add(p);
            }

            var result = new Point3DCollection(after.Count);

            foreach (var q in after)
            {
                var qCell = ToCell(q, cellSize);
                bool hasMatch = false;

                // Check this cell and all 26 neighbors
                for (int dx = -1; dx <= 1 && !hasMatch; dx++)
                {
                    for (int dy = -1; dy <= 1 && !hasMatch; dy++)
                    {
                        for (int dz = -1; dz <= 1 && !hasMatch; dz++)
                        {
                            var neighborKey = (qCell.ix + dx, qCell.iy + dy, qCell.iz + dz);
                            if (!grid.TryGetValue(neighborKey, out var list))
                                continue;

                            foreach (var p in list)
                            {
                                double dxp = p.X - q.X;
                                double dyp = p.Y - q.Y;
                                double dzp = p.Z - q.Z;
                                double distSq = dxp * dxp + dyp * dyp + dzp * dzp;

                                if (distSq <= radiusSq)
                                {
                                    hasMatch = true;
                                    break;
                                }
                            }
                        }
                    }
                }

                // If no close neighbor in "before", treat it as new object point
                if (!hasMatch)
                {
                    result.Add(q);
                }
            }

            return result;
        }

        /// <summary>
        /// Maps a 3D point into integer voxel coordinates.
        /// </summary>
        private static (int ix, int iy, int iz) ToCell(Point3D p, double cellSize)
        {
            int ix = (int)Math.Floor(p.X / cellSize);
            int iy = (int)Math.Floor(p.Y / cellSize);
            int iz = (int)Math.Floor(p.Z / cellSize);
            return (ix, iy, iz);
        }
    }
}