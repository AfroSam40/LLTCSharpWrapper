public static PointCloud ClipToRectangleXY(PointCloud cloud, RectangleF rect)
{
    if (cloud?.Points == null || cloud.Points.Count == 0)
        return new PointCloud { Points = new List<ScanPointXYZ>(0) };

    float left   = rect.X - rect.Width * 0.5f;
    float top    = rect.Y - rect.Height * 0.5f;
    float right  = left + rect.Width;
    float bottom = top + rect.Height;

    var src = cloud.Points;
    int count = src.Count;

    var dst = new List<ScanPointXYZ>();

    for (int i = 0; i < count; i++)
    {
        var p = src[i];
        if (p.X >= left && p.X <= right && p.Y >= top && p.Y <= bottom)
            dst.Add(p);
    }

    return new PointCloud { Points = dst };
}