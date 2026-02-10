using System;
using System.Collections.Generic;
using System.Windows.Media.Media3D;

public static class PointCloudQueries
{
    public static List<double> PeakZ(Point3DCollection cloud, IList<Point3D> queries)
    {
        var outZ = new List<double>(queries.Count);
        for (int i = 0; i < queries.Count; i++)
        {
            var q = queries[i]; int bi = 0; double bd = double.PositiveInfinity;
            for (int j = 0; j < cloud.Count; j++) { var p = cloud[j]; double dx = p.X - q.X, dy = p.Y - q.Y, d = dx*dx + dy*dy; if (d < bd) { bd = d; bi = j; } }
            outZ.Add(cloud[bi].Z);
        }
        return outZ;
    }
}
```0