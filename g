using ILGPU;
using ILGPU.Runtime;
using System;
using System.Collections.Generic;
using System.Drawing;
using System.Linq;

public struct ScanPointXYZ
{
    public float X;
    public float Y;
    public float Z;

    public ScanPointXYZ(float x, float y, float z)
    {
        X = x;
        Y = y;
        Z = z;
    }
}

public sealed class PointCloud
{
    public List<ScanPointXYZ> Points { get; set; } = new();
}

public static class PointCloudGpu
{
    // Marks 1 if point is inside rect, else 0
    static void ClipToRectangleKernel(
        Index1D i,
        ArrayView<ScanPointXYZ> points,
        ArrayView<byte> keepMask,
        float left,
        float top,
        float right,
        float bottom)
    {
        if (i >= points.Length)
            return;

        var p = points[i];

        keepMask[i] = (p.X >= left && p.X <= right && p.Y >= top && p.Y <= bottom)
            ? (byte)1
            : (byte)0;
    }

    public static PointCloud ClipToRectangleXY(
        Context context,
        Accelerator accelerator,
        PointCloud cloud,
        RectangleF rect)
    {
        if (cloud == null || cloud.Points == null || cloud.Points.Count == 0)
            return new PointCloud();

        // Convert center-based rect to top-left-based rect if needed
        rect = new RectangleF(
            rect.X - rect.Width / 2f,
            rect.Y - rect.Height / 2f,
            rect.Width,
            rect.Height);

        var input = cloud.Points.ToArray();
        var mask = new byte[input.Length];

        using var dPoints = accelerator.Allocate1D(input);
        using var dMask = accelerator.Allocate1D<byte>(input.Length);

        var kernel = accelerator.LoadAutoGroupedStreamKernel<
            Index1D,
            ArrayView<ScanPointXYZ>,
            ArrayView<byte>,
            float, float, float, float>(ClipToRectangleKernel);

        float left = rect.Left;
        float top = rect.Top;
        float right = rect.Right;
        float bottom = rect.Bottom;

        kernel(input.Length, dPoints.View, dMask.View, left, top, right, bottom);
        accelerator.Synchronize();

        dMask.CopyToCPU(mask);

        var result = new List<ScanPointXYZ>();
        result.Capacity = mask.Count(m => m == 1);

        for (int i = 0; i < input.Length; i++)
        {
            if (mask[i] == 1)
                result.Add(input[i]);
        }

        return new PointCloud { Points = result };
    }
}