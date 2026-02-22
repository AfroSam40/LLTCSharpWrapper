using System;
using System.Numerics;

public static class TransformCompare
{
    /// <summary>
    /// Returns a printable summary comparing refined ICP transform to the initial guess.
    /// Delta = refined * inverse(initial).
    /// </summary>
    public static string CompareToInitialGuessString(
        pointmatcher.net.EuclideanTransform initial,
        pointmatcher.net.EuclideanTransform refined,
        int decimals = 6)
    {
        static string F(float v, int d) => v.ToString("F" + d);
        static string D(double v, int d) => v.ToString("F" + d);

        static double RotationAngleDeg(Quaternion q)
        {
            q = Quaternion.Normalize(q);
            double w = Math.Clamp(q.W, -1.0f, 1.0f);
            return (2.0 * Math.Acos(w)) * 180.0 / Math.PI;
        }

        static string Tf(string name, pointmatcher.net.EuclideanTransform T, int d)
        {
            var q = Quaternion.Normalize(T.rotation);
            var t = T.translation;
            return $"{name}: t=({F(t.X,d)},{F(t.Y,d)},{F(t.Z,d)})  q=({F(q.X,d)},{F(q.Y,d)},{F(q.Z,d)},{F(q.W,d)})";
        }

        var delta = refined * initial.Inverse();
        var dt = delta.translation;

        double dtMag = dt.Length();
        double dAng = RotationAngleDeg(delta.rotation);

        return string.Join(Environment.NewLine, new[]
        {
            Tf("Init", initial, decimals),
            Tf("ICP ", refined, decimals),
            Tf("Δ   ", delta, decimals),
            $"Δ translation = ({F(dt.X,decimals)},{F(dt.Y,decimals)},{F(dt.Z,decimals)}), |Δt|={D(dtMag,decimals)}",
            $"Δ rotation    = {D(dAng,decimals)} deg"
        });
    }
}
```0