using System;
using System.Collections.Generic;
using System.Linq;
using System.Windows.Media.Media3D;

public static class VoidFinder
{
    /// <summary>
    /// Finds centers of "voids" (holes) on a surface plane using only the plane's inlier points.
    /// Uses auto pixel sizing based on point spacing, fills occupancy, flood-fills outside, and
    /// finds enclosed empty regions whose equivalent radius >= minVoidRadius.
    /// Returns centers as world Point3D on the plane.
    /// </summary>
    public static List<Point3D> FindVoidCenters(PlaneFitResult plane, double minVoidRadius)
    {
        var pts3 = plane?.InlierPoints;
        if (pts3 == null || pts3.Count < 50 || minVoidRadius <= 0) return new List<Point3D>();

        // --- Build an orthonormal (U,V,N) basis for the plane ---
        Vector3D n = plane.Normal; if (n.LengthSquared < 1e-12) n = new Vector3D(0, 0, 1);
        n.Normalize();
        Vector3D u = Vector3D.CrossProduct(n, Math.Abs(n.Z) < 0.9 ? new Vector3D(0, 0, 1) : new Vector3D(0, 1, 0));
        if (u.LengthSquared < 1e-12) u = new Vector3D(1, 0, 0);
        u.Normalize();
        Vector3D v = Vector3D.CrossProduct(n, u); v.Normalize();
        Point3D c0 = plane.Centroid;

        // --- Project 3D points to plane UV (2D) ---
        var uv = new (double U, double V)[pts3.Count];
        double uMin = double.PositiveInfinity, vMin = double.PositiveInfinity;
        double uMax = double.NegativeInfinity, vMax = double.NegativeInfinity;

        for (int i = 0; i < pts3.Count; i++)
        {
            var d = pts3[i] - c0;
            double uu = Vector3D.DotProduct(d, u);
            double vv = Vector3D.DotProduct(d, v);
            uv[i] = (uu, vv);
            if (uu < uMin) uMin = uu; if (uu > uMax) uMax = uu;
            if (vv < vMin) vMin = vv; if (vv > vMax) vMax = vv;
        }

        // --- Auto-pick pixel size from point spacing (fast estimate) ---
        // Sample ~512 points, estimate nearest-neighbor distance by local binning
        int sample = Math.Min(512, uv.Length);
        int step = Math.Max(1, uv.Length / sample);

        // coarse bin to estimate spacing
        double spanU = uMax - uMin, spanV = vMax - vMin;
        if (spanU <= 1e-9 || spanV <= 1e-9) return new List<Point3D>();

        int bins = 64;
        double bu = spanU / bins, bv = spanV / bins;
        var buckets = new Dictionary<long, List<int>>(sample * 2);

        long Key(int iu, int iv) => ((long)iu << 32) ^ (uint)iv;

        for (int i = 0; i < uv.Length; i += step)
        {
            int iu = (int)((uv[i].U - uMin) / bu);
            int iv = (int)((uv[i].V - vMin) / bv);
            iu = Math.Clamp(iu, 0, bins - 1);
            iv = Math.Clamp(iv, 0, bins - 1);
            long k = Key(iu, iv);
            if (!buckets.TryGetValue(k, out var list)) buckets[k] = list = new List<int>(8);
            list.Add(i);
        }

        double EstimateNN()
        {
            double sum = 0; int cnt = 0;
            foreach (var kv in buckets)
            {
                var a = kv.Value;
                foreach (int idx in a)
                {
                    // check same + neighbor bins
                    int iu = (int)((uv[idx].U - uMin) / bu);
                    int iv = (int)((uv[idx].V - vMin) / bv);
                    double best2 = double.PositiveInfinity;

                    for (int du = -1; du <= 1; du++)
                    for (int dv = -1; dv <= 1; dv++)
                    {
                        int ju = iu + du, jv = iv + dv;
                        if (ju < 0 || ju >= bins || jv < 0 || jv >= bins) continue;
                        if (!buckets.TryGetValue(Key(ju, jv), out var b)) continue;

                        for (int t = 0; t < b.Count; t++)
                        {
                            int j = b[t];
                            if (j == idx) continue;
                            double dx = uv[idx].U - uv[j].U;
                            double dy = uv[idx].V - uv[j].V;
                            double d2 = dx * dx + dy * dy;
                            if (d2 < best2) best2 = d2;
                        }
                    }
                    if (best2 < double.PositiveInfinity)
                    {
                        sum += Math.Sqrt(best2);
                        cnt++;
                        if (cnt >= 256) return sum / cnt; // early exit
                    }
                }
            }
            return cnt > 0 ? sum / cnt : Math.Max(spanU, spanV) / 200.0;
        }

        double nn = EstimateNN();
        double pixel = Math.Max(nn / 1.5, 1e-6);          // auto-close gaps
        int stampR = Math.Clamp((int)Math.Ceiling(nn / pixel), 1, 3); // small brush (fast)

        // --- Build grid with margin ---
        double pad = Math.Max(minVoidRadius * 1.5, nn * 4);
        uMin -= pad; uMax += pad; vMin -= pad; vMax += pad;
        int W = (int)Math.Ceiling((uMax - uMin) / pixel);
        int H = (int)Math.Ceiling((vMax - vMin) / pixel);
        if (W < 32 || H < 32 || W > 8000 || H > 8000) return new List<Point3D>(); // safety

        // 0 = empty, 1 = occupied (material), 2 = outside-marked
        var grid = new byte[W * H];

        int Idx(int x, int y) => y * W + x;

        // stamp occupied cells around each point (small disk)
        for (int i = 0; i < uv.Length; i++)
        {
            int x0 = (int)((uv[i].U - uMin) / pixel);
            int y0 = (int)((uv[i].V - vMin) / pixel);
            if ((uint)x0 >= (uint)W || (uint)y0 >= (uint)H) continue;

            for (int dy = -stampR; dy <= stampR; dy++)
            {
                int y = y0 + dy; if ((uint)y >= (uint)H) continue;
                int dxMax = (int)Math.Floor(Math.Sqrt(stampR * stampR - dy * dy));
                int xs = Math.Max(0, x0 - dxMax);
                int xe = Math.Min(W - 1, x0 + dxMax);
                int row = y * W;
                for (int x = xs; x <= xe; x++) grid[row + x] = 1;
            }
        }

        // --- Flood fill from borders to mark "outside" empty space (2) ---
        var q = new Queue<int>(W * 2 + H * 2);

        void EnqueueIfEmpty(int x, int y)
        {
            int id = Idx(x, y);
            if (grid[id] == 0) { grid[id] = 2; q.Enqueue(id); }
        }

        for (int x = 0; x < W; x++) { EnqueueIfEmpty(x, 0); EnqueueIfEmpty(x, H - 1); }
        for (int y = 0; y < H; y++) { EnqueueIfEmpty(0, y); EnqueueIfEmpty(W - 1, y); }

        while (q.Count > 0)
        {
            int id = q.Dequeue();
            int y = id / W;
            int x = id - y * W;

            if (x > 0)       EnqueueIfEmpty(x - 1, y);
            if (x + 1 < W)   EnqueueIfEmpty(x + 1, y);
            if (y > 0)       EnqueueIfEmpty(x, y - 1);
            if (y + 1 < H)   EnqueueIfEmpty(x, y + 1);
        }

        // --- Remaining empty (0) regions are holes: find their centroids + area ---
        var centers = new List<Point3D>();
        int minAreaPx = (int)Math.Ceiling(Math.PI * (minVoidRadius / pixel) * (minVoidRadius / pixel));

        var visited = new byte[W * H]; // 0/1
        var q2 = new Queue<int>();

        for (int start = 0; start < grid.Length; start++)
        {
            if (grid[start] != 0 || visited[start] != 0) continue;

            // BFS region
            visited[start] = 1;
            q2.Enqueue(start);

            int area = 0;
            double sumX = 0, sumY = 0;

            while (q2.Count > 0)
            {
                int id = q2.Dequeue();
                int y = id / W;
                int x = id - y * W;

                area++;
                sumX += x + 0.5;
                sumY += y + 0.5;

                void Push(int nx, int ny)
                {
                    if ((uint)nx >= (uint)W || (uint)ny >= (uint)H) return;
                    int nid = Idx(nx, ny);
                    if (grid[nid] == 0 && visited[nid] == 0)
                    {
                        visited[nid] = 1;
                        q2.Enqueue(nid);
                    }
                }

                Push(x - 1, y);
                Push(x + 1, y);
                Push(x, y - 1);
                Push(x, y + 1);
            }

            if (area < minAreaPx) continue;

            // Equivalent radius filter (area in real units)
            double areaReal = area * pixel * pixel;
            double eqR = Math.Sqrt(areaReal / Math.PI);
            if (eqR < minVoidRadius) continue;

            // Convert centroid grid->UV->world
            double cxPx = sumX / area;
            double cyPx = sumY / area;
            double Uc = uMin + cxPx * pixel;
            double Vc = vMin + cyPx * pixel;

            Point3D centerWorld = c0 + (Uc * u) + (Vc * v);
            centers.Add(centerWorld);
        }

        return centers;
    }
}