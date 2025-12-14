<Window
    ...
    xmlns:hx="http://helixToolkit.github.io/wpf/SharpDX">

    <Window.Resources>
        <!-- SharpDX viewports need an EffectsManager -->
        <hx:DefaultEffectsManager x:Key="EffectsManager" />
    </Window.Resources>

    <hx:Viewport3DX x:Name="Viewport"
                    EffectsManager="{StaticResource EffectsManager}"
                    ZoomExtentsWhenLoaded="True">

        <!-- Optional but recommended: define a camera explicitly -->
        <hx:Viewport3DX.Camera>
            <hx:PerspectiveCamera Position="0,0,10"
                                  LookDirection="0,0,-10"
                                  UpDirection="0,1,0"
                                  FarPlaneDistance="5000" />
        </hx:Viewport3DX.Camera>

        <!-- "DefaultLights" equivalent -->
        <hx:SunLight />

        <!-- Mesh -->
        <hx:MeshGeometryModel3D x:Name="MeshModel"
                                Geometry="{Binding MeshGeometry}"
                                Material="{Binding MeshMaterial}" />

        <!-- Point cloud -->
        <hx:PointGeometryModel3D x:Name="PointCloudPoints"
                                 Geometry="{Binding PointCloudGeometry}"
                                 Color="DodgerBlue"
                                 Size="1.5" />

        <!-- Projected points -->
        <hx:PointGeometryModel3D x:Name="ProjectedPoints"
                                 Geometry="{Binding ProjectedPointsGeometry}"
                                 Color="Red"
                                 Size="1.5" />

    </hx:Viewport3DX>
</Window>


----- 

using HelixToolkit.Wpf.SharpDX;
using SharpDX;
using System.Linq;

// points: IEnumerable<System.Windows.Media.Media3D.Point3D> or your own point type
PointCloudGeometry = new PointGeometry3D
{
    Positions = new Vector3Collection(points.Select(p => new Vector3((float)p.X, (float)p.Y, (float)p.Z)))
};

// same idea for projected points
ProjectedPointsGeometry = new PointGeometry3D
{
    Positions = new Vector3Collection(projected.Select(p => new Vector3((float)p.X, (float)p.Y, (float)p.Z)))
};

-----

using HelixToolkit.Wpf.SharpDX;
using SharpDX;
using System.Windows.Media.Media3D;

public static PointGeometry3D ToPointGeometry(Point3DCollection pts)
{
    var positions = new Vector3Collection(pts.Count);
    foreach (var p in pts)
        positions.Add(new Vector3((float)p.X, (float)p.Y, (float)p.Z));

    return new PointGeometry3D { Positions = positions };
}