using OxyPlot;
using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Media.Media3D;

namespace LLT
{
    public struct PlaneBasis
    {
        public Point3D Origin;   // a point on the plane
        public Vector3D U;       // in-plane axis 1 (normalized)
        public Vector3D V;       // in-plane axis 2 (normalized)
        public Vector3D Normal;  // plane normal (normalized)
    }

    public enum ViewFace
    {
        Front,   // XZ plane   (look along ±Y)
        Back,    // same projection as Front
        Left,    // same projection as Right
        Right,   // YZ plane   (look along ±X)
        Top,     // XY plane   (look along ±Z)
        Bottom   // same projection as Top
    }

    public static class PointCloudProcessing
    {
        public static Point3DCollection VoxelDownSample(Point3DCollection input, double cellSize)
        {
            if (input == null || input.Count == 0)
                return input;

            var result = new Point3DCollection();
            var seen = new HashSet<(int, int, int)>();

            foreach (var p in input)
            {
                int ix = (int)Math.Floor(p.X / cellSize);
                int iy = (int)Math.Floor(p.Y / cellSize);
                int iz = (int)Math.Floor(p.Z / cellSize);

                var key = (ix, iy, iz);
                if (seen.Add(key))
                    result.Add(p);
            }

            return result;
        }


        /// <summary>
        /// Build a plane/basis from 3 points (e.g. a triangle).
        /// </summary>
        public static PlaneBasis CreatePlaneFromTriangle(Point3D p0, Point3D p1, Point3D p2)
        {
            var e1 = p1 - p0;
            var e2 = p2 - p0;

            if (e1.LengthSquared < 1e-12 || e2.LengthSquared < 1e-12)
                throw new ArgumentException("Triangle edges are degenerate.");

            var n = Vector3D.CrossProduct(e1, e2); // normal
            if (n.LengthSquared < 1e-12)
                throw new ArgumentException("Triangle points are colinear, cannot form a plane.");

            n.Normalize();

            var u = e1;
            u.Normalize();

            var v = Vector3D.CrossProduct(n, u);
            v.Normalize();

            return new PlaneBasis
            {
                Origin = p0,
                U = u,
                V = v,
                Normal = n
            };
        }

        /// <summary>
        /// Project a 3D point into 2D coordinates in a given plane basis.
        /// </summary>
        public static Point ProjectPointToPlane2D(Point3D point, PlaneBasis plane)
        {
            Vector3D vec = point - plane.Origin;

            double x = Vector3D.DotProduct(vec, plane.U);
            double y = Vector3D.DotProduct(vec, plane.V);

            return new Point(x, y);
        }

        /// <summary>
        /// Collect all vertex positions from a Model3D hierarchy.
        /// (Transforms are ignored here for simplicity.
        /// If you need transforms, we can extend this.)
        /// </summary>
        public static List<Point3D> CollectAllPositions(Model3D model)
        {
            var result = new List<Point3D>();
            CollectAllPositionsInternal(model, result);
            return result;
        }

        private static void CollectAllPositionsInternal(Model3D model, List<Point3D> points)
        {
            if (model is Model3DGroup group)
            {
                foreach (var child in group.Children)
                    CollectAllPositionsInternal(child, points);
            }
            else if (model is GeometryModel3D geom &&
                     geom.Geometry is MeshGeometry3D mesh)
            {
                foreach (var p in mesh.Positions)
                    points.Add(p);
            }
        }

        /// <summary>
        /// Find the first mesh in the model that has triangles.
        /// Returns the mesh and its 3 vertices for triangleIndex.
        /// </summary>
        public static bool TryGetTriangleFromFirstMesh(
            Model3D model,
            int triangleIndex,
            out MeshGeometry3D mesh,
            out Point3D p0,
            out Point3D p1,
            out Point3D p2)
        {
            mesh = null;
            p0 = p1 = p2 = new Point3D();

            MeshGeometry3D foundMesh = FindFirstMesh(model);
            if (foundMesh == null || foundMesh.TriangleIndices == null ||
                foundMesh.TriangleIndices.Count < 3)
            {
                return false;
            }

            int baseIndex = triangleIndex * 3;
            if (baseIndex + 2 >= foundMesh.TriangleIndices.Count)
                return false;

            int i0 = foundMesh.TriangleIndices[baseIndex + 0];
            int i1 = foundMesh.TriangleIndices[baseIndex + 1];
            int i2 = foundMesh.TriangleIndices[baseIndex + 2];

            if (i0 < 0 || i0 >= foundMesh.Positions.Count ||
                i1 < 0 || i1 >= foundMesh.Positions.Count ||
                i2 < 0 || i2 >= foundMesh.Positions.Count)
            {
                return false;
            }

            mesh = foundMesh;
            p0 = foundMesh.Positions[i0];
            p1 = foundMesh.Positions[i1];
            p2 = foundMesh.Positions[i2];
            return true;
        }

        private static MeshGeometry3D FindFirstMesh(Model3D model)
        {
            if (model is GeometryModel3D gm && gm.Geometry is MeshGeometry3D mg)
                return mg;

            if (model is Model3DGroup group)
            {
                foreach (var child in group.Children)
                {
                    var result = FindFirstMesh(child);
                    if (result != null)
                        return result;
                }
            }

            return null;
        }

        /// <summary>
        /// Main helper:
        ///   - Finds a triangle in the first mesh
        ///   - Builds a plane from that triangle
        ///   - Projects all vertices in the Model3D into that plane
        ///   - Returns OxyPlot DataPoints ready for plotting.
        /// </summary>
        public static List<DataPoint> ProjectModelToFacePlane(Model3D model, int triangleIndex = 0)
        {
            if (model == null)
                throw new ArgumentNullException(nameof(model));

            if (!TryGetTriangleFromFirstMesh(model, triangleIndex, out var mesh, out var p0, out var p1, out var p2))
                throw new InvalidOperationException("Could not find a valid mesh/triangle in the model.");

            // Build plane from that triangle
            var plane = CreatePlaneFromTriangle(p0, p1, p2);

            // Collect all positions in the model
            var points3D = CollectAllPositions(model);

            // Project to 2D
            var result = new List<DataPoint>(points3D.Count);
            foreach (var p in points3D)
            {
                var uv = ProjectPointToPlane2D(p, plane);
                result.Add(new DataPoint(uv.X, uv.Y));
            }

            return result;
        }

        public static List<DataPoint> ProjectModelToFacePlane(Point3DCollection points3D, PlaneBasis plane)
        {
            if (points3D == null)
                throw new ArgumentNullException(nameof(points3D));

            var result = new List<DataPoint>(points3D.Count);
            foreach (var p in points3D)
            {
                var uv = ProjectPointToPlane2D(p, plane);
                result.Add(new DataPoint(uv.X, uv.Y));
            }

            return result;
        }

        public static List<DataPoint> ProjectModelToFacePlane(
            Point3DCollection points3D,
            Point3D p0,
            Point3D p1,
            Point3D p2)
        {
            if (points3D == null)
                throw new ArgumentNullException(nameof(points3D));

            var plane = CreatePlaneFromTriangle(p0, p1, p2);
            return ProjectModelToFacePlane(points3D, plane);
        }

        /// <summary>
        /// Orthogonally project a 3D point onto the plane.
        /// </summary>
        public static Point3D ProjectPointToPlane3D(Point3D point, PlaneBasis plane)
        {
            var vec = point - plane.Origin;
            double dist = Vector3D.DotProduct(vec, plane.Normal); // signed distance to plane
            // Move the point back along the normal
            return point - dist * plane.Normal;
        }

        /// <summary>
        /// Project a list of 3D points onto the plane and return a Point3DCollection
        /// </summary>
        public static Point3DCollection ProjectPointsToPlane3D(
            IEnumerable<Point3D> points,
            PlaneBasis plane)
        {
            var result = new Point3DCollection();
            foreach (var p in points)
            {
                result.Add(ProjectPointToPlane3D(p, plane));
            }
            return result;
        }


        public static Point3DCollection ProjectPointsToPlane3D(IEnumerable<Point3D> points3D, Point3D p0, Point3D p1, Point3D p2)
        {
            if (points3D == null)
                throw new ArgumentNullException(nameof(points3D));

            var plane = CreatePlaneFromTriangle(p0, p1, p2);

            var result = new Point3DCollection();
            foreach (var p in points3D)
            {
                result.Add(ProjectPointToPlane3D(p, plane));
            }
            return result;
        }

        public static List<DataPoint> ProjectToFace(Point3DCollection points, ViewFace face)
        {
            var result = new List<DataPoint>(points.Count);

            foreach (var p in points)
            {
                switch (face)
                {
                    case ViewFace.Front:
                    case ViewFace.Back:
                        // Looking along ±Y -> keep X (horizontal) and Z (vertical)
                        result.Add(new DataPoint(p.X, p.Z));
                        break;

                    case ViewFace.Right:
                    case ViewFace.Left:
                        // Looking along ±X -> keep Y and Z
                        result.Add(new DataPoint(p.Y, p.Z));
                        break;

                    case ViewFace.Top:
                    case ViewFace.Bottom:
                        // Looking along ±Z -> keep X and Y
                        result.Add(new DataPoint(p.X, p.Y));
                        break;
                }
            }

            return result;
        }

        public static Point3DCollection ProjectToFace3D(Point3DCollection points, ViewFace face)
        {
            var result = new Point3DCollection(points.Count);

            foreach (var p in points)
            {
                switch (face)
                {
                    case ViewFace.Front:
                    case ViewFace.Back:
                        // Looking along Y → squash Y → XZ plane
                        result.Add(new Point3D(p.X, 0.0, p.Z));
                        break;

                    case ViewFace.Right:
                    case ViewFace.Left:
                        // Looking along X → squash X → YZ plane
                        result.Add(new Point3D(0.0, p.Y, p.Z));
                        break;

                    case ViewFace.Top:
                    case ViewFace.Bottom:
                        // Looking along Z → squash Z → XY plane
                        result.Add(new Point3D(p.X, p.Y, 0.0));
                        break;
                }
            }

            return result;
        }
    }
}
