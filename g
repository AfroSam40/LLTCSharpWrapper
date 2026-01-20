using System;
using OpenCvSharp;

public static class PoseExtract
{
    // Returns Euler angles (Rx,Ry,Rz) in radians using ZYX convention:
    // R = Rz(Rz) * Ry(Ry) * Rx(Rx)
    public static (double Rx, double Ry, double Rz, double Tx, double Ty, double Tz)
        ExtractRxyzTxyz(Mat T, bool enforceRigid = true)
    {
        if (T == null) throw new ArgumentNullException(nameof(T));
        if (!((T.Rows == 3 && T.Cols == 4) || (T.Rows == 4 && T.Cols == 4)))
            throw new ArgumentException("T must be 3x4 or 4x4.", nameof(T));

        // Ensure CV_64F
        Mat Td;
        if (T.Type() == MatType.CV_64F) Td = T;
        else
        {
            Td = new Mat();
            T.ConvertTo(Td, MatType.CV_64F);
        }

        // Extract R (3x3)
        var R = new Mat(3, 3, MatType.CV_64F);
        Td[new Rect(0, 0, 3, 3)].CopyTo(R);

        // Extract t
        double tx = Td.At<double>(0, 3);
        double ty = Td.At<double>(1, 3);
        double tz = Td.At<double>(2, 3);

        if (enforceRigid)
            R = NearestProperRotation(R);

        // Euler ZYX
        double r00 = R.At<double>(0, 0), r01 = R.At<double>(0, 1), r02 = R.At<double>(0, 2);
        double r10 = R.At<double>(1, 0), r11 = R.At<double>(1, 1), r12 = R.At<double>(1, 2);
        double r20 = R.At<double>(2, 0), r21 = R.At<double>(2, 1), r22 = R.At<double>(2, 2);

        double ry = Math.Asin(Clamp(-r20, -1.0, 1.0));
        double cy = Math.Cos(ry);

        double rx, rz;
        if (Math.Abs(cy) > 1e-12)
        {
            rx = Math.Atan2(r21, r22); // about X
            rz = Math.Atan2(r10, r00); // about Z
        }
        else
        {
            // Gimbal lock
            rx = 0.0;
            rz = Math.Atan2(-r01, r11);
        }

        return (rx, ry, rz, tx, ty, tz);
    }

    private static Mat NearestProperRotation(Mat A)
    {
        var w = new Mat();
        var u = new Mat();
        var vt = new Mat();
        Cv2.SVDecomp(A, w, u, vt);

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

    private static double Clamp(double v, double lo, double hi) =>
        v < lo ? lo : (v > hi ? hi : v);
}