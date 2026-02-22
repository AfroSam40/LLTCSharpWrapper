using System;
using System.Numerics;

public static class RtCompare
{
    /// <summary>
    /// Compares refined vs init.
    /// Returns a printable string with:
    ///  - Δ = refined ∘ inv(init)
    ///  - Δ translation (and magnitude)
    ///  - Δ rotation angle (deg)
    /// Assumes Rt maps src->dst (same direction for both).
    /// </summary>
    public static string CompareRefinedWithInitString(in FastIcp3D.Rt init, in FastIcp3D.Rt refined, int decimals = 6)
    {
        // Build 4x4 matrices
        static Matrix4x4 ToM(in FastIcp3D.Rt t) => new Matrix4x4(
            (float)t.r00, (float)t.r01, (float)t.r02, 0f,
            (float)t.r10, (float)t.r11, (float)t.r12, 0f,
            (float)t.r20, (float)t.r21, (float)t.r22, 0f,
            (float)t.tx,  (float)t.ty,  (float)t.tz,  1f);

        static string F(float v, int d) => v.ToString("F" + d);
        static string D(double v, int d) => v.ToString("F" + d);

        static double AngleDeg(Quaternion q)
        {
            q = Quaternion.Normalize(q);
            double w = Math.Clamp(q.W, -1.0f, 1.0f);
            return (2.0 * Math.Acos(w)) * 180.0 / Math.PI;
        }

        var Mi = ToM(in init);
        var Mr = ToM(in refined);

        if (!Matrix4x4.Invert(Mi, out var MiInv))
            return "Init matrix not invertible.";

        // Δ = refined * inv(init)
        var Md = Mr * MiInv;

        var deltaT = new Vector3(Md.M41, Md.M42, Md.M43);
        double deltaTmag = deltaT.Length();

        var qd = Quaternion.CreateFromRotationMatrix(Md);
        double deltaAng = AngleDeg(qd);

        string RtStr(string name, in FastIcp3D.Rt t) =>
            $"{name}: t=({D(t.tx,decimals)},{D(t.ty,decimals)},{D(t.tz,decimals)})  " +
            $"R=[{D(t.r00,decimals)} {D(t.r01,decimals)} {D(t.r02,decimals)}; " +
            $"{D(t.r10,decimals)} {D(t.r11,decimals)} {D(t.r12,decimals)}; " +
            $"{D(t.r20,decimals)} {D(t.r21,decimals)} {D(t.r22,decimals)}]";

        return string.Join(Environment.NewLine, new[]
        {
            RtStr("Init   ", in init),
            RtStr("Refined", in refined),
            $"Δt = ({F(deltaT.X,decimals)},{F(deltaT.Y,decimals)},{F(deltaT.Z,decimals)}), |Δt|={D(deltaTmag,decimals)}",
            $"ΔR angle = {D(deltaAng,decimals)} deg"
        });
    }
}
```0