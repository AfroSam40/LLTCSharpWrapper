Viewport.EffectsManager = new HelixToolkit.Wpf.SharpDX.DefaultEffectsManager();

Viewport.Camera = new HelixToolkit.Wpf.SharpDX.PerspectiveCamera
{
    Position = new System.Windows.Media.Media3D.Point3D(0, 0, 500),
    LookDirection = new System.Windows.Media.Media3D.Vector3D(0, 0, -500),
    UpDirection = new System.Windows.Media.Media3D.Vector3D(0, 1, 0),
    FarPlaneDistance = 1e6
};