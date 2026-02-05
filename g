using System;
using System.Linq;
using System.Windows.Media;
using System.Windows.Media.Media3D;
using HelixToolkit.Wpf.SharpDX;
using SharpDX; // Color4, Vector3, Vector2

public static class WpfToSharpDx
{
    /// <summary>
    /// Clears dst.Children and populates it from a WPF Model3DGroup by converting MeshGeometry3D -> MeshGeometryModel3D.
    /// NOTE: WPF Model3DGroup cannot be assigned directly to GroupModel3D.
    /// </summary>
    public static void SetFromModel3DGroup(GroupModel3D dst, Model3DGroup src)
    {
        if (dst == null) throw new ArgumentNullException(nameof(dst));
        if (src == null) throw new ArgumentNullException(nameof(src));

        dst.Children.Clear();

        void Recurse(Model3D model, Matrix3D parent)
        {
            if (model == null) return;

            // Accumulate transforms
            var m = parent;
            if (model.Transform != null)
            {
                var t = model.Transform.Value;
                m.Append(t);
            }

            // Nested group
            if (model is Model3DGroup g)
            {
                foreach (var child in g.Children)
                    Recurse(child, m);
                return;
            }

            // Mesh geometry
            if (model is GeometryModel3D gm && gm.Geometry is MeshGeometry3D wpfMesh)
            {
                // Convert geometry
                var dxMesh = new HelixToolkit.Wpf.SharpDX.MeshGeometry3D
                {
                    Positions = new Vector3Collection(wpfMesh.Positions.Select(p => new Vector3((float)p.X, (float)p.Y, (float)p.Z))),
                    Indices = new IntCollection(wpfMesh.TriangleIndices),
                };

                if (wpfMesh.Normals != null && wpfMesh.Normals.Count == wpfMesh.Positions.Count)
                    dxMesh.Normals = new Vector3Collection(wpfMesh.Normals.Select(n => new Vector3((float)n.X, (float)n.Y, (float)n.Z)));

                if (wpfMesh.TextureCoordinates != null && wpfMesh.TextureCoordinates.Count == wpfMesh.Positions.Count)
                    dxMesh.TextureCoordinates = new Vector2Collection(wpfMesh.TextureCoordinates.Select(uv => new Vector2((float)uv.X, (float)uv.Y)));

                // Convert material (basic diffuse only)
                var mat = new PhongMaterial();
                if (gm.Material is DiffuseMaterial dm && dm.Brush is SolidColorBrush scb)
                {
                    var c = scb.Color;
                    mat.DiffuseColor = new Color4(c.R / 255f, c.G / 255f, c.B / 255f, c.A / 255f);
                    mat.AmbientColor = mat.DiffuseColor * 0.2f;
                }
                else
                {
                    mat.DiffuseColor = new Color4(0.7f, 0.7f, 0.7f, 1.0f);
                    mat.AmbientColor = mat.DiffuseColor * 0.2f;
                }

                var dxModel = new MeshGeometryModel3D
                {
                    Geometry = dxMesh,
                    Material = mat,
                    // Keep WPF transform type; SharpDX Helix uses WPF Transform3D on Element3D
                    Transform = new MatrixTransform3D(m),
                };

                dst.Children.Add(dxModel);
            }

            // (Optional) you can add conversions for WPF lights here if your src contains them.
        }

        Recurse(src, Matrix3D.Identity);
    }
}