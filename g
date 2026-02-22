using System;
using System.Collections.Generic;
using System.Numerics;
using MathNet.Numerics.LinearAlgebra;

public static class RigidFit3D
{
    /// <summary>
    /// Finds best-fit transform mapping src -> dst in least-squares sense.
    /// If withScale=true: Umeyama similarity (uniform scale + R + t).
    /// If withScale=false: Kabsch rigid (R + t).
    /// Returns (r00..r22, tx,ty,tz, s) where s=1 for rigid.
    /// </summary>
    public static (double r00, double r01, double r02,
                   double r10, double r11, double r12,
                   double r20, double r21, double r22,
                   double tx,  double ty,  double tz,
                   double s)
        KabschUmeyama(IReadOnlyList<Vector3> src, IReadOnlyList<Vector3> dst, bool withScale = false)
    {
        if (src is null || dst is null) throw new ArgumentNullException();
        if (src.Count != dst.Count) throw new ArgumentException("src/dst must have same length.");
        if (src.Count < 3) throw new ArgumentException("Need at least 3 point pairs.");

        int n = src.Count;

        // Centroids
        Vector3 cs = default, cd = default;
        for (int i = 0; i < n; i++) { cs += src[i]; cd += dst[i]; }
        cs /= n; cd /= n;

        // Build covariance and variance
        // H = sum( (src-cs)(dst-cd)^T )
        var H = Matrix<double>.Build.Dense(3, 3);
        double varSrc = 0.0;

        for (int i = 0; i < n; i++)
        {
            var xs = src[i] - cs;
            var yd = dst[i] - cd;

            H[0, 0] += xs.X * yd.X; H[0, 1] += xs.X * yd.Y; H[0, 2] += xs.X * yd.Z;
            H[1, 0] += xs.Y * yd.X; H[1, 1] += xs.Y * yd.Y; H[1, 2] += xs.Y * yd.Z;
            H[2, 0] += xs.Z * yd.X; H[2, 1] += xs.Z * yd.Y; H[2, 2] += xs.Z * yd.Z;

            varSrc += xs.X * xs.X + xs.Y * xs.Y + xs.Z * xs.Z;
        }

        // SVD: H = U * S * V^T
        var svd = H.Svd(true);
        var U = svd.U;
        var Vt = svd.VT;
        var V = Vt.Transpose();

        // R = V * U^T (maps src -> dst)
        var R = V * U.Transpose();

        // Enforce proper rotation (no reflection)
        if (R.Determinant() < 0)
        {
            // Flip last column of V, then recompute R
            V[0, 2] *= -1; V[1, 2] *= -1; V[2, 2] *= -1;
            R = V * U.Transpose();
        }

        double s = 1.0;
        if (withScale)
        {
            // Umeyama scale: s = trace(S) / varSrc
            // trace(S) = sum singular values
            double traceS = svd.S.Sum();
            s = traceS / varSrc; // uniform scale
        }

        // t = cd - s*R*cs
        var csV = Vector<double>.Build.Dense(new[] { (double)cs.X, (double)cs.Y, (double)cs.Z });
        var cdV = Vector<double>.Build.Dense(new[] { (double)cd.X, (double)cd.Y, (double)cd.Z });
        var tV = cdV - (s * (R * csV));

        return (
            R[0, 0], R[0, 1], R[0, 2],
            R[1, 0], R[1, 1], R[1, 2],
            R[2, 0], R[2, 1], R[2, 2],
            tV[0], tV[1], tV[2],
            s
        );
    }
}


public static ScanPointXYZ Transform3DPointFast(
    in ScanPointXYZ p,
    in (double r00, double r01, double r02,
        double r10, double r11, double r12,
        double r20, double r21, double r22,
        double tx,  double ty,  double tz,
        double s) rt)
{
    double x = p.X, y = p.Y, z = p.Z;
    double sx = rt.s;

    return new ScanPointXYZ(
        sx * (rt.r00 * x + rt.r01 * y + rt.r02 * z) + rt.tx,
        sx * (rt.r10 * x + rt.r11 * y + rt.r12 * z) + rt.ty,
        sx * (rt.r20 * x + rt.r21 * y + rt.r22 * z) + rt.tz
    );
}
