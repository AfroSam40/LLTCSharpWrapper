using System;
using System.Drawing;
using System.Threading.Tasks;

public struct ScanPointXYZ
{
    public float X;
    public float Y;
    public float Z;
}

public sealed class PointCloud
{
    public ScanPointXYZ[] Points { get; set; } = Array.Empty<ScanPointXYZ>();
}

public static class PointCloudClipper
{
    public static PointCloud ClipToRectangleXY(PointCloud cloud, RectangleF rect)
    {
        var src = cloud?.Points;
        if (src == null || src.Length == 0)
            return new PointCloud { Points = Array.Empty<ScanPointXYZ>() };

        float left   = rect.X - rect.Width * 0.5f;
        float top    = rect.Y - rect.Height * 0.5f;
        float right  = left + rect.Width;
        float bottom = top + rect.Height;

        int count = src.Length;
        int workers = Environment.ProcessorCount;
        int chunkSize = (count + workers - 1) / workers;

        var counts = new int[workers];

        // Pass 1: count survivors per chunk
        Parallel.For(0, workers, w =>
        {
            int start = w * chunkSize;
            int end = Math.Min(start + chunkSize, count);
            if (start >= end) return;

            int localCount = 0;

            for (int i = start; i < end; i++)
            {
                var p = src[i];
                if (p.X >= left && p.X <= right && p.Y >= top && p.Y <= bottom)
                    localCount++;
            }

            counts[w] = localCount;
        });

        // Prefix sum to get write offsets
        var offsets = new int[workers];
        int total = 0;
        for (int w = 0; w < workers; w++)
        {
            offsets[w] = total;
            total += counts[w];
        }

        if (total == 0)
            return new PointCloud { Points = Array.Empty<ScanPointXYZ>() };

        var dst = new ScanPointXYZ[total];

        // Pass 2: write directly into each thread's assigned slice
        Parallel.For(0, workers, w =>
        {
            int start = w * chunkSize;
            int end = Math.Min(start + chunkSize, count);
            if (start >= end) return;

            int write = offsets[w];

            for (int i = start; i < end; i++)
            {
                var p = src[i];
                if (p.X >= left && p.X <= right && p.Y >= top && p.Y <= bottom)
                    dst[write++] = p;
            }
        });

        return new PointCloud { Points = dst };
    }
}