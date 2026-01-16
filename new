using System;
using OpenCvSharp;

public static class PoseExtract
{
    /// <summary>
    /// Extracts pose as (Rx, Ry, Rz, Tx, Ty, Tz) from a 3x4 or 4x4 transform.
    /// Rx/Ry/Rz are Euler angles in radians using ZYX convention:
    /// R = Rz(Rz) * Ry(Ry) * Rx(Rx)  (yaw, pitch, roll)
    /// If enforceRigid=true, the 3x3 block is projected to nearest proper rotation (det=+1).
    /// </summary>
    public static (double Rx, double Ry, double Rz, double Tx, double Ty, double Tz)
        ExtractRxyzTxyz(Mat T, bool enforceRigid = true)
    {
        if (T == null) throw new ArgumentNullException(nameof(T));
        if (!((T.Rows == 3 && T.Cols == 4) || (T.Rows == 4 && T.Cols == 4)))
            throw new ArgumentException("T must be 3x4 or 4x4.", nameof(T));

        // Ensure CV_64F
        Mat Td = T.Type() == MatType.CV_64F ? T : T.Clone().ConvertTo(new Mat(), MatType.CV_64F);

        // Extract R (3x3)
        var R = new Mat(3, 3, MatType.CV_64F);
        Td[new Rect(0, 0, 3, 3)].CopyTo(R);

        // Extract t
        double tx = Td.At<double>(0, 3);
        double ty = Td.At<double>(1, 3);
        double tz = Td.At<double>(2, 3);

        if (enforceRigid)
            R = NearestProperRotation(R);

        // Euler ZYX: R = Rz * Ry * Rx
        // ry (pitch) = asin(-r20)
        double r00 = R.At<double>(0, 0), r01 = R.At<double>(0, 1), r02 = R.At<double>(0, 2);
        double r10 = R.At<double>(1, 0), r11 = R.At<double>(1, 1), r12 = R.At<double>(1, 2);
        double r20 = R.At<double>(2, 0), r21 = R.At<double>(2, 1), r22 = R.At<double>(2, 2);

        double ry = Math.Asin(Clamp(-r20, -1.0, 1.0));
        double cy = Math.Cos(ry);

        double rx, rz;
        if (Math.Abs(cy) > 1e-12)
        {
            rx = Math.Atan2(r21, r22); // roll about X
            rz = Math.Atan2(r10, r00); // yaw about Z
        }
        else
        {
            // Gimbal lock: ry ≈ ±90°, choose rx = 0 and compute rz from remaining terms
            rx = 0.0;
            rz = Math.Atan2(-r01, r11);
        }

        return (rx, ry, rz, tx, ty, tz);
    }

    private static Mat NearestProperRotation(Mat A)
    {
        Cv2.SVDecomp(A, out _, out Mat u, out Mat vt);
        Mat R = u * vt;

        if (Cv2.Determinant(R) < 0.0)
        {
            // Flip last column of U to force det=+1
            u.Set(0, 2, -u.At<double>(0, 2));
            u.Set(1, 2, -u.At<double>(1, 2));
            u.Set(2, 2, -u.At<double>(2, 2));
            R = u * vt;
        }

        return R;
    }

    private static double Clamp(double v, double lo, double hi) => v < lo ? lo : (v > hi ? hi : v);
}