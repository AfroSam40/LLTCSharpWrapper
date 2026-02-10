using HelixToolkit.Wpf.SharpDX;
using SharpDX;
using System.Windows;

namespace LLT
{
    public partial class MainWindow : Window
    {
        public MainWindow()
        {
            InitializeComponent();

            // Drop a test marker so you can confirm you can see it
            SetMarkerSphere(0, 0, 0, 2f);     // center
            ZoomToScene();
        }

        public void SetMarkerSphere(float x, float y, float z, float radius = 1f)
        {
            // Build geometry (small sphere)
            var mb = new MeshBuilder(false, false);
            mb.AddSphere(new Vector3(x, y, z), radius, 16, 16);
            var geo = mb.ToMeshGeometry3D();

            MarkerModel.Geometry = geo;

            // PhongMaterial is the “diffuse-like” equivalent you want in SharpDX
            MarkerModel.Material = new PhongMaterial
            {
                DiffuseColor = new Color4(1f, 0f, 0f, 1f),  // red
                AmbientColor = new Color4(1f, 0f, 0f, 1f),
                SpecularColor = new Color4(0f, 0f, 0f, 1f),
                ReflectiveColor = new Color4(0f, 0f, 0f, 1f)
            };

            MarkerModel.IsRendering = true;
        }

        public void ZoomToScene()
        {
            // ZoomExtents exists on Viewport3DX
            // If you have clipping issues, bump FarPlaneDistance in XAML camera.
            View3DX.ZoomExtents();
        }
    }
}