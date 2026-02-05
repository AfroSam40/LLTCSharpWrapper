using HelixToolkit.Wpf.SharpDX;
using SharpDX;

public static class GeoBounds
{
    public static BoundingBox ComputeBoundingBox(Vector3Collection positions)
    {
        if (positions == null || positions.Count == 0)
            return new BoundingBox(new Vector3(0, 0, 0), new Vector3(0, 0, 0));

        var min = new Vector3(float.PositiveInfinity, float.PositiveInfinity, float.PositiveInfinity);
        var max = new Vector3(float.NegativeInfinity, float.NegativeInfinity, float.NegativeInfinity);

        for (int i = 0; i < positions.Count; i++)
        {
            var p = positions[i];
            min.X = Math.Min(min.X, p.X);
            min.Y = Math.Min(min.Y, p.Y);
            min.Z = Math.Min(min.Z, p.Z);

            max.X = Math.Max(max.X, p.X);
            max.Y = Math.Max(max.Y, p.Y);
            max.Z = Math.Max(max.Z, p.Z);
        }

        return new BoundingBox(min, max);
    }
}