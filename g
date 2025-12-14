using System.Windows.Input;
using HelixToolkit.Wpf;
using System.Windows.Media.Media3D;

// in your MainWindow ctor after InitializeComponent():
public MainWindow()
{
    InitializeComponent();
    Viewport.MouseDown += Viewport_MouseDown;
}

private void Viewport_MouseDown(object sender, MouseButtonEventArgs e)
{
    var pos2D = e.GetPosition(Viewport.Viewport); // Viewport.Viewport is the underlying Viewport3D

    // Find hits at this screen position
    var hits = Viewport3DHelper.FindHits(Viewport.Viewport, pos2D);
    if (hits == null || hits.Count == 0)
        return;

    // Take nearest hit
    var hit = hits[0];
    Point3D p = hit.Position;

    // For example, show in a status label or MessageBox
    // (replace CoordinatesLabel with your actual control)
    CoordinatesLabel.Content = $"X={p.X:F3}, Y={p.Y:F3}, Z={p.Z:F3}";
    // or
    // MessageBox.Show($"X={p.X}, Y={p.Y}, Z={p.Z}");
}

--------

using System;
using System.Collections.Generic;
using System.Windows.Media.Media3D;

public static class PointCloudClustering3D
{
    /// <summary>
    /// Runs a simple DBSCAN-style clustering on a Point3DCollection
    /// and returns the largest cluster as a new Point3DCollection.
    ///
    /// eps  = neighborhood radius (same units as your points, e.g. mm)
    /// minPts = minimum number of neighbors (including the point itself)
    ///          to be considered a core point.
    /// </summary>
    public static Point3DCollection GetLargestDbscanCluster(
        Point3DCollection points,
        double eps,
        int minPts)
    {
        if (points == null) throw new ArgumentNullException(nameof(points));
        if (points.Count == 0) return new Point3DCollection();
        if (eps <= 0) throw new ArgumentOutOfRangeException(nameof(eps));
        if (minPts <= 0) throw new ArgumentOutOfRangeException(nameof(minPts));

        int n = points.Count;
        // 0 = unvisited, -1 = noise, >0 = cluster id
        int[] labels = new int[n];
        int clusterId = 0;
        double eps2 = eps * eps;

        var neighbors = new List<int>();

        for (int i = 0; i < n; i++)
        {
            if (labels[i] != 0)
                continue; // already visited

            GetNeighbors(points, i, eps2, neighbors);
            if (neighbors.Count < minPts)
            {
                labels[i] = -1; // noise
                continue;
            }

            clusterId++;
            ExpandCluster(points, i, neighbors, clusterId, eps2, minPts, labels);
        }

        if (clusterId == 0)
            return new Point3DCollection(); // nothing clustered

        // Count sizes per clusterId
        var clusterCounts = new int[clusterId + 1];
        for (int i = 0; i < n; i++)
        {
            int id = labels[i];
            if (id > 0) clusterCounts[id]++;
        }

        // Find largest cluster id
        int bestId = 1;
        int bestCount = clusterCounts[1];
        for (int id = 2; id <= clusterId; id++)
        {
            if (clusterCounts[id] > bestCount)
            {
                bestCount = clusterCounts[id];
                bestId = id;
            }
        }

        // Collect points for largest cluster
        var result = new Point3DCollection(bestCount);
        for (int i = 0; i < n; i++)
        {
            if (labels[i] == bestId)
                result.Add(points[i]);
        }

        return result;
    }

    private static void GetNeighbors(
        Point3DCollection points,
        int index,
        double eps2,
        List<int> neighbors)
    {
        neighbors.Clear();
        var p = points[index];

        for (int j = 0; j < points.Count; j++)
        {
            var q = points[j];
            double dx = p.X - q.X;
            double dy = p.Y - q.Y;
            double dz = p.Z - q.Z;
            double dist2 = dx * dx + dy * dy + dz * dz;

            if (dist2 <= eps2)
                neighbors.Add(j);
        }
    }

    private static void ExpandCluster(
        Point3DCollection points,
        int seedIndex,
        List<int> neighbors,
        int clusterId,
        double eps2,
        int minPts,
        int[] labels)
    {
        labels[seedIndex] = clusterId;

        int i = 0;
        while (i < neighbors.Count)
        {
            int idx = neighbors[i];

            if (labels[idx] == -1)
            {
                // previously marked noise -> border point
                labels[idx] = clusterId;
            }

            if (labels[idx] == 0)
            {
                labels[idx] = clusterId;

                var neighbors2 = new List<int>();
                GetNeighbors(points, idx, eps2, neighbors2);

                if (neighbors2.Count >= minPts)
                {
                    // merge neighbors2 into neighbors (simple dedupe)
                    foreach (var nb in neighbors2)
                    {
                        if (!neighbors.Contains(nb))
                            neighbors.Add(nb);
                    }
                }
            }

            i++;
        }
    }
}