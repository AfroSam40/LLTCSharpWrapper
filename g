using HelixToolkit.Wpf.SharpDX;
using SharpDX;
using System.Linq;
using System.Windows.Media.Media3D;

public static void SetPointCloud(PointGeometryModel3D model, Point3DCollection pts)
{
    if (model == null) throw new ArgumentNullException(nameof(model));
    if (pts == null || pts.Count == 0)
    {
        model.Geometry = null;
        return;
    }

    var positions = new Vector3Collection(pts.Count);
    foreach (var p in pts)
        positions.Add(new Vector3((float)p.X, (float)p.Y, (float)p.Z));

    model.Geometry = new PointGeometry3D
    {
        Positions = positions
        // Colors = optional (see below)
    };

    // If you want a constant color for all points:
    model.Material = new ColorMaterial { Color = Color4.DodgerBlue };
}