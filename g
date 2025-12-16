using System;
using System.Collections.Generic;
using System.Linq;
using System.Windows.Media;
using System.Windows.Media.Media3D;
using HelixToolkit.Wpf;

namespace FiducialDetection
{
    // Simple plane result – adapt this to your existing PlaneFitResult if you already have one.
    public struct PlaneFitResult
    {
        public Point3D Origin;   // A point on the plane (e.g. centroid)
        public Vector3D Normal;  // Normal, should be normalized
    }

    public class CornerReliefDetection
    {
        public Point3D Corner3D { get; set; }        // Square-corner / circle center (on plane)
        public Point3D CircleCenter3D { get; set; }  // Same as Corner3D here
        public double Radius { get; set; }           // Fitted radius
        public double ArcCoverageDeg { get; set; }   // Angular extent of detected arc (deg)
        public List<Point3D> ArcPoints3D { get; set; } = new List<Point3D>();
    }

    internal class ProjectedPoint
    {
        public double U;
        public double V;
        public double Height;        // Signed distance along plane normal
        public Point3D Original3D;

        // DBSCAN state
        public int ClusterId = -1;
        public bool Visited = false;
    }

    public static class CornerReliefDetector
    {
        /// <summary>
        /// Detect "dog-bone" circular corner reliefs in a plate with square through-holes.
        /// 
        /// Steps:
        /// 1) Project to plate plane.
        /// 2) Threshold by height to isolate hole interior points.
        /// 3) DBSCAN in 2D to get one cluster per hole.
        /// 4) For each cluster, build convex hull, treat hull vertices as corner candidates.
        /// 5) Around each candidate, fit a circle (center fixed to corner, radius = mean distance)
        ///    and check arc coverage and residual.
        /// </summary>
        /// <param name="points">Full plate point cloud.</param>
        /// <param name="plane">Best-fit plate plane.</param>
        /// <param name="holeDepthThreshold">
        /// Negative height (in mm) below the plate plane to consider a point as part of a through-hole.
        /// If your plane normal is flipped, you may need to invert the sign or use Abs.
        /// </param>
        /// <param name="expectedRadius">
        /// Expected radius of the corner relief (mm). Used to select arc candidates around each corner.
        /// </param>
        /// <param name="radiusBand">
        /// Fractional tolerance around expected radius; e.g. 0.3 means [0.7R, 1.3R].
        /// </param>
        /// <param name="clusterEps">
        /// DBSCAN neighborhood radius (in mm in the projected plane).
        /// </param>
        /// <param name="clusterMinPoints">
        /// DBSCAN minimum points per cluster.
        /// </param>
        /// <param name="cornerCenterTolerance">
        /// Not used in this version (center is forced to corner), but kept for future refinement.
        /// </param>
        /// <param name="minArcCoverageDeg">
        /// Minimum arc span in degrees to accept (e.g. 60 for a quarter circle).
        /// </param>
        /// <param name="maxArcCoverageDeg">
        /// Maximum arc span in degrees to accept (e.g. 120 for a quarter circle).
        /// </param>
        /// <param name="maxRadiusRmsError">
        /// Max RMS deviation of distances from fitted radius (in mm).
        /// </param>
        public static List<CornerReliefDetection> DetectCornerReliefs(
            Point3DCollection points,
            PlaneFitResult plane,
            double holeDepthThreshold,
            double expectedRadius,
            double radiusBand = 0.3,
            double clusterEps = 0.15,
            int clusterMinPoints = 50,
            double cornerCenterTolerance = 0.2,
            double minArcCoverageDeg = 60,
            double maxArcCoverageDeg = 120,
            double maxRadiusRmsError = 0.05)
        {
            if (points == null || points.Count == 0)
                return new List<CornerReliefDetection>();

            // 1) Build plane basis (Origin, n, in-plane u,v)
            BuildPlaneBasis(plane, out Point3D origin, out Vector3D n, out Vector3D uAxis, out Vector3D vAxis);

            // 2) Project all points and select "hole" points by height
            var projected = new List<ProjectedPoint>(points.Count);
            for (int i = 0; i < points.Count; i++)
            {
                var p = points[i];
                Vector3D op = p - origin;

                double u = Vector3D.DotProduct(op, uAxis);
                double v = Vector3D.DotProduct(op, vAxis);
                double h = Vector3D.DotProduct(op, n); // signed distance along normal

                projected.Add(new ProjectedPoint
                {
                    U = u,
                    V = v,
                    Height = h,
                    Original3D = p
                });
            }

            // IMPORTANT: we assume hole points lie "below" the plane 
            // (negative Height). If not, flip the condition or the plane normal.
            var holePts = projected.Where(pp => pp.Height < -holeDepthThreshold).ToList();

            if (holePts.Count == 0)
                return new List<CornerReliefDetection>();

            // 3) DBSCAN in (U,V) to get one cluster per hole
            var clusters = Dbscan2D(holePts, clusterEps, clusterMinPoints);

            var detections = new List<CornerReliefDetection>();

            // 4) For each cluster, build convex hull and test corners
            foreach (var cluster in clusters)
            {
                if (cluster.Count < 10)
                    continue;

                var hull = ConvexHull(cluster);
                if (hull.Count < 3)
                    continue;

                double rMin = expectedRadius * (1.0 - radiusBand);
                double rMax = expectedRadius * (1.0 + radiusBand);

                foreach (var corner in hull)
                {
                    // 4a) Collect candidate arc points near the expected radius
                    var arcCandidates = new List<ProjectedPoint>();
                    foreach (var p in cluster)
                    {
                        double du = p.U - corner.U;
                        double dv = p.V - corner.V;
                        double r = Math.Sqrt(du * du + dv * dv);
                        if (r >= rMin && r <= rMax)
                            arcCandidates.Add(p);
                    }

                    if (arcCandidates.Count < 20)
                        continue;

                    // 4b) Fit radius with center fixed to this corner
                    double cx = corner.U;
                    double cy = corner.V;
                    var radii = arcCandidates.Select(p =>
                    {
                        double du = p.U - cx;
                        double dv = p.V - cy;
                        return Math.Sqrt(du * du + dv * dv);
                    }).ToArray();

                    double R = radii.Average();
                    double rms = Math.Sqrt(radii.Select(r => (r - R) * (r - R)).Average());

                    if (rms > maxRadiusRmsError)
                        continue; // circle doesn't fit well

                    // 4c) Compute arc coverage
                    var angles = arcCandidates
                        .Select(p => Math.Atan2(p.V - cy, p.U - cx))
                        .ToArray();

                    NormalizeAngles(angles, out double arcSpanDeg);
                    if (arcSpanDeg < minArcCoverageDeg || arcSpanDeg > maxArcCoverageDeg)
                        continue;

                    // 4d) Build detection
                    var center3D = FromUV(cx, cy, origin, uAxis, vAxis);

                    var det = new CornerReliefDetection
                    {
                        Corner3D = center3D,
                        CircleCenter3D = center3D,
                        Radius = R,
                        ArcCoverageDeg = arcSpanDeg,
                        ArcPoints3D = arcCandidates.Select(pp => pp.Original3D).ToList()
                    };

                    detections.Add(det);
                }
            }

            return detections;
        }

        /// <summary>
        /// Build HelixToolkit visuals (green arcs + labels) for each detection
        /// and add them to the given viewport.
        /// </summary>
        public static void CreateCornerReliefVisuals(
            IEnumerable<CornerReliefDetection> detections,
            PlaneFitResult plane,
            HelixViewport3D viewport,
            double arcRadiusScale = 1.0,
            int arcSegments = 64)
        {
            if (viewport == null || detections == null)
                return;

            BuildPlaneBasis(plane, out Point3D origin, out Vector3D n, out Vector3D uAxis, out Vector3D vAxis);

            foreach (var det in detections)
            {
                // Re-project center into (u,v) frame (just in case)
                Vector3D oc = det.CircleCenter3D - origin;
                double uc = Vector3D.DotProduct(oc, uAxis);
                double vc = Vector3D.DotProduct(oc, vAxis);
                double R = det.Radius * arcRadiusScale;

                var arcPoints = new Point3DCollection();
                for (int i = 0; i <= arcSegments; i++)
                {
                    double t = 2.0 * Math.PI * i / arcSegments;
                    double u = uc + R * Math.Cos(t);
                    double v = vc + R * Math.Sin(t);

                    arcPoints.Add(FromUV(u, v, origin, uAxis, vAxis));
                }

                var arcVisual = new LinesVisual3D
                {
                    Points = arcPoints,
                    Thickness = 1.5,
                    Color = Colors.LimeGreen
                };
                viewport.Children.Add(arcVisual);

                var text = new BillboardTextVisual3D
                {
                    Text = $"R={det.Radius:F3} mm",
                    Position = det.CircleCenter3D + n * 0.3, // lift a bit off the surface
                    Foreground = Brushes.Yellow
                };
                viewport.Children.Add(text);
            }
        }

        #region Helpers

        private static void BuildPlaneBasis(
            PlaneFitResult plane,
            out Point3D origin,
            out Vector3D n,
            out Vector3D uAxis,
            out Vector3D vAxis)
        {
            origin = plane.Origin;
            n = plane.Normal;
            n.Normalize();

            // Pick an arbitrary vector not parallel to n
            Vector3D temp = Math.Abs(n.Z) < 0.9 ? new Vector3D(0, 0, 1) : new Vector3D(0, 1, 0);

            uAxis = Vector3D.CrossProduct(temp, n);
            if (uAxis.LengthSquared < 1e-12)
                uAxis = Vector3D.CrossProduct(new Vector3D(1, 0, 0), n);

            uAxis.Normalize();
            vAxis = Vector3D.CrossProduct(n, uAxis);
            vAxis.Normalize();
        }

        private static Point3D FromUV(double u, double v, Point3D origin, Vector3D uAxis, Vector3D vAxis)
        {
            // origin + u*uAxis + v*vAxis
            var p = origin + uAxis * u;
            p += vAxis * v;
            return p;
        }

        /// <summary>
        /// Simple DBSCAN on 2D (U,V) for ProjectedPoint.
        /// </summary>
        private static List<List<ProjectedPoint>> Dbscan2D(List<ProjectedPoint> points, double eps, int minPts)
        {
            var clusters = new List<List<ProjectedPoint>>();
            int clusterId = 0;

            foreach (var p in points)
            {
                if (p.Visited)
                    continue;

                p.Visited = true;
                var neighbors = RegionQuery(points, p, eps);

                if (neighbors.Count < minPts)
                    continue; // noise

                var cluster = new List<ProjectedPoint>();
                ExpandCluster(points, p, neighbors, cluster, clusterId, eps, minPts);
                clusters.Add(cluster);
                clusterId++;
            }

            return clusters;
        }

        private static void ExpandCluster(
            List<ProjectedPoint> points,
            ProjectedPoint p,
            List<ProjectedPoint> neighbors,
            List<ProjectedPoint> cluster,
            int clusterId,
            double eps,
            int minPts)
        {
            p.ClusterId = clusterId;
            cluster.Add(p);

            var queue = new Queue<ProjectedPoint>(neighbors);
            while (queue.Count > 0)
            {
                var q = queue.Dequeue();
                if (!q.Visited)
                {
                    q.Visited = true;
                    var neighbors2 = RegionQuery(points, q, eps);
                    if (neighbors2.Count >= minPts)
                    {
                        foreach (var n in neighbors2)
                        {
                            if (n.ClusterId < 0)
                                queue.Enqueue(n);
                        }
                    }
                }
                if (q.ClusterId < 0)
                {
                    q.ClusterId = clusterId;
                    cluster.Add(q);
                }
            }
        }

        private static List<ProjectedPoint> RegionQuery(List<ProjectedPoint> points, ProjectedPoint p, double eps)
        {
            double eps2 = eps * eps;
            var neighbors = new List<ProjectedPoint>();
            foreach (var q in points)
            {
                double du = q.U - p.U;
                double dv = q.V - p.V;
                if (du * du + dv * dv <= eps2)
                    neighbors.Add(q);
            }
            return neighbors;
        }

        /// <summary>
        /// Convex hull via monotone chain in (U,V).
        /// Returns hull vertices in counter-clockwise order.
        /// </summary>
        private static List<ProjectedPoint> ConvexHull(List<ProjectedPoint> points)
        {
            if (points.Count <= 1)
                return new List<ProjectedPoint>(points);

            var sorted = points
                .OrderBy(p => p.U)
                .ThenBy(p => p.V)
                .ToList();

            var lower = new List<ProjectedPoint>();
            foreach (var p in sorted)
            {
                while (lower.Count >= 2 && Cross(lower[lower.Count - 2], lower[lower.Count - 1], p) <= 0)
                    lower.RemoveAt(lower.Count - 1);
                lower.Add(p);
            }

            var upper = new List<ProjectedPoint>();
            for (int i = sorted.Count - 1; i >= 0; i--)
            {
                var p = sorted[i];
                while (upper.Count >= 2 && Cross(upper[upper.Count - 2], upper[upper.Count - 1], p) <= 0)
                    upper.RemoveAt(upper.Count - 1);
                upper.Add(p);
            }

            lower.RemoveAt(lower.Count - 1);
            upper.RemoveAt(upper.Count - 1);
            lower.AddRange(upper);
            return lower;
        }

        private static double Cross(ProjectedPoint a, ProjectedPoint b, ProjectedPoint c)
        {
            // cross product of AB x AC in 2D
            double abx = b.U - a.U;
            double aby = b.V - a.V;
            double acx = c.U - a.U;
            double acy = c.V - a.V;
            return abx * acy - aby * acx;
        }

        /// <summary>
        /// Normalize angle list to get a robust arc span in degrees.
        /// Handles wrap-around at -π/π by trying an offset.
        /// </summary>
        private static void NormalizeAngles(double[] angles, out double arcSpanDeg)
        {
            if (angles.Length == 0)
            {
                arcSpanDeg = 0;
                return;
            }

            Array.Sort(angles);
            double span1 = angles[angles.Length - 1] - angles[0];

            // Try shifting all angles by +2π if they are negative
            var shifted = angles.Select(a => (a < 0 ? a + 2.0 * Math.PI : a)).ToArray();
            Array.Sort(shifted);
            double span2 = shifted[shifted.Length - 1] - shifted[0];

            double span = Math.Min(span1, span2);
            arcSpanDeg = span * 180.0 / Math.PI;
        }

        #endregion
    }
}