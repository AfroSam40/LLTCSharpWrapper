using System;
using System.Numerics;
using System.Runtime.CompilerServices;

public static class PitchRefine
{
    public static ((double rx, double ry, double rz, double tx, double ty, double tz) refined, double bestMaxAbsDz)
        RefineRyMinimizeMaxDzAcrossX(
            ReadOnlySpan<Vector3> reference,
            ReadOnlySpan<Vector3> reading,
            (double rx, double ry, double rz, double tx, double ty, double tz) rt,
            int bins = 200,
            double searchHalfRangeRad = 0.01,   // ~0.57°
            int coarseSteps = 41,
            int refineIters = 20,
            int strideRef = 4,
            int strideRead = 4)
    {
        if (reference.Length < 10 || reading.Length < 10) return (rt, double.PositiveInfinity);
        bins = Math.Max(10, bins);

        // Find X range (reference frame) for binning
        float xMin = float.PositiveInfinity, xMax = float.NegativeInfinity;
        for (int i = 0; i < reference.Length; i += Math.Max(1, strideRef))
        {
            float x = reference[i].X;
            if (x < xMin) xMin = x;
            if (x > xMax) xMax = x;
        }
        if (!(xMax > xMin)) return (rt, double.PositiveInfinity);

        float invBin = (float)(bins / (double)(xMax - xMin));

        // Precompute reference mean Z per X-bin
        var refSum = new double[bins];
        var refCnt = new int[bins];
        for (int i = 0; i < reference.Length; i += Math.Max(1, strideRef))
        {
            var p = reference[i];
            int b = (int)((p.X - xMin) * invBin);
            if ((uint)b >= (uint)bins) continue;
            refSum[b] += p.Z;
            refCnt[b]++;
        }

        var readSum = new double[bins];
        var readCnt = new int[bins];

        // Precompute sin/cos terms that do not change while refining ry
        double cx = Math.Cos(rt.rx), sx = Math.Sin(rt.rx);
        double cz = Math.Cos(rt.rz), sz = Math.Sin(rt.rz);

        // Evaluate objective for a candidate ry: max |Δz| across X-bins where both clouds have support
        double Eval(double ryCand)
        {
            double cy = Math.Cos(ryCand), sy = Math.Sin(ryCand);

            // R = Rz * Ry * Rx (ZYX yaw-pitch-roll)
            double r00 = cz * cy;
            double r01 = cz * sy * sx - sz * cx;
            double r02 = cz * sy * cx + sz * sx;

            double r10 = sz * cy;
            double r11 = sz * sy * sx + cz * cx;
            double r12 = sz * sy * cx - cz * sx;

            double r20 = -sy;
            double r21 = cy * sx;
            double r22 = cy * cx;

            Array.Clear(readSum, 0, bins);
            Array.Clear(readCnt, 0, bins);

            // Transform reading points, then bin by X' (in reference frame), accumulate Z'
            for (int i = 0; i < reading.Length; i += Math.Max(1, strideRead))
            {
                var p = reading[i];
                double x = p.X, y = p.Y, z = p.Z;

                double xp = r00 * x + r01 * y + r02 * z + rt.tx;
                double zp = r20 * x + r21 * y + r22 * z + rt.tz;

                int b = (int)(((float)xp - xMin) * invBin);
                if ((uint)b >= (uint)bins) continue;

                readSum[b] += zp;
                readCnt[b]++;
            }

            double maxAbs = 0.0;
            int used = 0;

            for (int b = 0; b < bins; b++)
            {
                if (refCnt[b] < 3 || readCnt[b] < 3) continue;

                double dz = (readSum[b] / readCnt[b]) - (refSum[b] / refCnt[b]);
                double a = Math.Abs(dz);
                if (a > maxAbs) maxAbs = a;
                used++;
            }

            return used < 5 ? double.PositiveInfinity : maxAbs;
        }

        // Coarse grid search around current ry
        double lo = rt.ry - searchHalfRangeRad;
        double hi = rt.ry + searchHalfRangeRad;

        double bestRy = rt.ry;
        double best = double.PositiveInfinity;

        if (coarseSteps < 3) coarseSteps = 3;
        for (int i = 0; i < coarseSteps; i++)
        {
            double ryCand = lo + (hi - lo) * i / (coarseSteps - 1);
            double val = Eval(ryCand);
            if (val < best) { best = val; bestRy = ryCand; }
        }

        // Narrow bracket around best grid point, then do golden-section refine
        double step = (hi - lo) / (coarseSteps - 1);
        lo = bestRy - step;
        hi = bestRy + step;

        const double gr = 0.6180339887498949;
        double a0 = lo, b0 = hi;
        double c = b0 - (b0 - a0) * gr;
        double d = a0 + (b0 - a0) * gr;
        double fc = Eval(c);
        double fd = Eval(d);

        for (int it = 0; it < Math.Max(1, refineIters); it++)
        {
            if (fc < fd)
            {
                b0 = d; d = c; fd = fc;
                c = b0 - (b0 - a0) * gr;
                fc = Eval(c);
            }
            else
            {
                a0 = c; c = d; fc = fd;
                d = a0 + (b0 - a0) * gr;
                fd = Eval(d);
            }
        }

        double ryFinal = (fc < fd) ? c : d;
        double scoreFinal = Math.Min(fc, fd);

        return ((rt.rx, ryFinal, rt.rz, rt.tx, rt.ty, rt.tz), scoreFinal);
    }
}