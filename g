using System;
using System.Collections.Generic;
using System.Numerics;
using System.Runtime.CompilerServices;

public static class FastIcp3D
{
    public readonly struct Rt
    {
        public readonly double r00, r01, r02, tx;
        public readonly double r10, r11, r12, ty;
        public readonly double r20, r21, r22, tz;

        public Rt(double r00, double r01, double r02, double tx,
                  double r10, double r11, double r12, double ty,
                  double r20, double r21, double r22, double tz)
        {
            this.r00 = r00; this.r01 = r01; this.r02 = r02; this.tx = tx;
            this.r10 = r10; this.r11 = r11; this.r12 = r12; this.ty = ty;
            this.r20 = r20; this.r21 = r21; this.r22 = r22; this.tz = tz;
        }

        [MethodImpl(MethodImplOptions.AggressiveInlining)]
        public Vector3 Apply(in Vector3 p)
        {
            double x = p.X, y = p.Y, z = p.Z;
            return new Vector3(
                (float)(r00 * x + r01 * y + r02 * z + tx),
                (float)(r10 * x + r11 * y + r12 * z + ty),
                (float)(r20 * x + r21 * y + r22 * z + tz)
            );
        }

        public static Rt Identity => new Rt(
            1,0,0,0,
            0,1,0,0,
            0,0,1,0);

        public Rt Compose(in Rt b) // this ∘ b : first b then this
        {
            // R = Ra*Rb, t = Ra*tb + ta
            double R00 = r00*b.r00 + r01*b.r10 + r02*b.r20;
            double R01 = r00*b.r01 + r01*b.r11 + r02*b.r21;
            double R02 = r00*b.r02 + r01*b.r12 + r02*b.r22;

            double R10 = r10*b.r00 + r11*b.r10 + r12*b.r20;
            double R11 = r10*b.r01 + r11*b.r11 + r12*b.r21;
            double R12 = r10*b.r02 + r11*b.r12 + r12*b.r22;

            double R20 = r20*b.r00 + r21*b.r10 + r22*b.r20;
            double R21 = r20*b.r01 + r21*b.r11 + r22*b.r21;
            double R22 = r20*b.r02 + r21*b.r12 + r22*b.r22;

            double Ttx = r00*b.tx + r01*b.ty + r02*b.tz + tx;
            double Tty = r10*b.tx + r11*b.ty + r12*b.tz + ty;
            double Ttz = r20*b.tx + r21*b.ty + r22*b.tz + tz;

            return new Rt(R00,R01,R02,Ttx, R10,R11,R12,Tty, R20,R21,R22,Ttz);
        }
    }

    // ------------------------------------------------------------
    // Public API
    // ------------------------------------------------------------
    /// <summary>
    /// Fast rigid ICP using a spatial hash grid for nearest neighbors.
    /// Returns refined Rt mapping src -> dst.
    /// </summary>
    public static Rt IcpRigidFast(
        ReadOnlySpan<Vector3> src,
        ReadOnlySpan<Vector3> dst,
        in Rt initial,
        float cellSize,
        float maxCorrDist,
        int maxIters = 30,
        float minDeltaRmse = 1e-5f,
        int maxPairs = 20000)
    {
        if (src.Length < 3 || dst.Length < 3) return initial;

        // Build spatial hash on dst once
        var grid = new Dictionary<long, List<int>>(dst.Length / 8);
        float inv = 1f / cellSize;

        for (int i = 0; i < dst.Length; i++)
        {
            var p = dst[i];
            var key = Key(p, inv);
            if (!grid.TryGetValue(key, out var list))
                grid[key] = list = new List<int>(8);
            list.Add(i);
        }

        // Work buffers (avoid allocs in loop)
        var pairsS = new Vector3[Math.Min(src.Length, maxPairs)];
        var pairsT = new Vector3[Math.Min(src.Length, maxPairs)];

        Rt T = initial;
        float prevRmse = float.PositiveInfinity;

        // Optional: stride sampling for huge clouds
        int stride = Math.Max(1, src.Length / Math.Max(1, maxPairs));

        for (int iter = 0; iter < maxIters; iter++)
        {
            int m = 0;
            double sumSq = 0.0;
            float maxD2 = maxCorrDist * maxCorrDist;

            // Gather correspondences using current transform
            for (int i = 0; i < src.Length && m < pairsS.Length; i += stride)
            {
                var s0 = src[i];
                var s = T.Apply(in s0);

                if (TryNearest(dst, grid, inv, s, out var t, out float d2) && d2 <= maxD2)
                {
                    pairsS[m] = s; // already transformed source point
                    pairsT[m] = t;
                    sumSq += d2;
                    m++;
                }
            }

            if (m < 3) break;

            float rmse = (float)Math.Sqrt(sumSq / m);
            if (Math.Abs(prevRmse - rmse) < minDeltaRmse) break;
            prevRmse = rmse;

            // Compute incremental rigid transform that maps current S -> T (Kabsch)
            var dT = BestRigidQuaternion(pairsS.AsSpan(0, m), pairsT.AsSpan(0, m));

            // Update: new T = dT ∘ T  (since pairsS were already in current frame)
            T = dT.Compose(in T);
        }

        return T;
    }

    // ------------------------------------------------------------
    // Nearest neighbor via spatial hash grid
    // ------------------------------------------------------------
    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static long Key(in Vector3 p, float invCell)
    {
        int ix = (int)MathF.Floor(p.X * invCell);
        int iy = (int)MathF.Floor(p.Y * invCell);
        int iz = (int)MathF.Floor(p.Z * invCell);
        return Pack(ix, iy, iz);
    }

    [MethodImpl(MethodImplOptions.AggressiveInlining)]
    private static long Pack(int x, int y, int z)
    {
        // 21 bits per axis pack (works well for typical metrology ranges; change if needed)
        const long mask = (1L << 21) - 1;
        return ((x & mask) << 42) | ((y & mask) << 21) | (z & mask);
    }

    private static bool TryNearest(ReadOnlySpan<Vector3> dst,
                                   Dictionary<long, List<int>> grid,
                                   float invCell,
                                   in Vector3 q,
                                   out Vector3 best,
                                   out float bestD2)
    {
        best = default;
        bestD2 = float.PositiveInfinity;

        int ix = (int)MathF.Floor(q.X * invCell);
        int iy = (int)MathF.Floor(q.Y * invCell);
        int iz = (int)MathF.Floor(q.Z * invCell);

        // Search 27 neighboring cells
        for (int dx = -1; dx <= 1; dx++)
        for (int dy = -1; dy <= 1; dy++)
        for (int dz = -1; dz <= 1; dz++)
        {
            long key = Pack(ix + dx, iy + dy, iz + dz);
            if (!grid.TryGetValue(key, out var list)) continue;

            for (int k = 0; k < list.Count; k++)
            {
                var p = dst[list[k]];
                var d = p - q;
                float d2 = Vector3.Dot(d, d);
                if (d2 < bestD2)
                {
                    bestD2 = d2;
                    best = p;
                }
            }
        }

        return bestD2 < float.PositiveInfinity;
    }

    // ------------------------------------------------------------
    // Kabsch (rigid) using Horn’s quaternion method (fast, no SVD lib)
    // Returns transform mapping S -> T
    // ------------------------------------------------------------
    private static Rt BestRigidQuaternion(ReadOnlySpan<Vector3> S, ReadOnlySpan<Vector3> T)
    {
        int n = S.Length;

        // Centroids
        Vector3 cs = default, ct = default;
        for (int i = 0; i < n; i++) { cs += S[i]; ct += T[i]; }
        cs /= n; ct /= n;

        // Cross-covariance components
        double Sxx=0,Sxy=0,Sxz=0, Syx=0,Syy=0,Syz=0, Szx=0,Szy=0,Szz=0;

        for (int i = 0; i < n; i++)
        {
            var a = S[i] - cs;
            var b = T[i] - ct;

            Sxx += a.X*b.X; Sxy += a.X*b.Y; Sxz += a.X*b.Z;
            Syx += a.Y*b.X; Syy += a.Y*b.Y; Syz += a.Y*b.Z;
            Szx += a.Z*b.X; Szy += a.Z*b.Y; Szz += a.Z*b.Z;
        }

        // Horn’s 4x4 N matrix (symmetric)
        double trace = Sxx + Syy + Szz;

        double n00 = trace;
        double n01 = Syz - Szy;
        double n02 = Szx - Sxz;
        double n03 = Sxy - Syx;

        double n11 = Sxx - Syy - Szz;
        double n12 = Sxy + Syx;
        double n13 = Szx + Sxz;

        double n22 = -Sxx + Syy - Szz;
        double n23 = Syz + Szy;

        double n33 = -Sxx - Syy + Szz;

        // Power iteration to get dominant eigenvector of N (very fast for 4x4)
        double q0=1, q1=0, q2=0, q3=0;
        for (int it = 0; it < 12; it++)
        {
            double v0 = n00*q0 + n01*q1 + n02*q2 + n03*q3;
            double v1 = n01*q0 + n11*q1 + n12*q2 + n13*q3;
            double v2 = n02*q0 + n12*q1 + n22*q2 + n23*q3;
            double v3 = n03*q0 + n13*q1 + n23*q2 + n33*q3;

            double invNorm = 1.0 / Math.Sqrt(v0*v0 + v1*v1 + v2*v2 + v3*v3);
            q0 = v0*invNorm; q1 = v1*invNorm; q2 = v2*invNorm; q3 = v3*invNorm;
        }

        // Quaternion (w,x,y,z) = (q0,q1,q2,q3)
        double w=q0, x=q1, y=q2, z=q3;

        // Rotation matrix from quaternion
        double ww=w*w, xx=x*x, yy=y*y, zz=z*z;
        double wx=w*x, wy=w*y, wz=w*z;
        double xy=x*y, xz=x*z, yz=y*z;

        double r00 = ww + xx - yy - zz;
        double r01 = 2*(xy - wz);
        double r02 = 2*(xz + wy);

        double r10 = 2*(xy + wz);
        double r11 = ww - xx + yy - zz;
        double r12 = 2*(yz - wx);

        double r20 = 2*(xz - wy);
        double r21 = 2*(yz + wx);
        double r22 = ww - xx - yy + zz;

        // t = ct - R*cs
        double tx = ct.X - (r00*cs.X + r01*cs.Y + r02*cs.Z);
        double ty = ct.Y - (r10*cs.X + r11*cs.Y + r12*cs.Z);
        double tz = ct.Z - (r20*cs.X + r21*cs.Y + r22*cs.Z);

        return new Rt(r00,r01,r02,tx, r10,r11,r12,ty, r20,r21,r22,tz);
    }
}