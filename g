using HelixToolkit.Wpf.SharpDX;

public MainWindow()
{
    InitializeComponent();

    Viewport.EffectsManager = new DefaultEffectsManager();
    Viewport.Camera = new PerspectiveCamera
    {
        Position = new System.Windows.Media.Media3D.Point3D(0, 0, 200),
        LookDirection = new System.Windows.Media.Media3D.Vector3D(0, 0, -200),
        UpDirection = new System.Windows.Media.Media3D.Vector3D(0, 1, 0),
        FarPlaneDistance = 100000
    };
}