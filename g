using OpenCvSharp;

public static ScanPointXYZ Transform3DPoint_RigidOnly(ScanPointXYZ p, Mat affine3x4)
{
    if (affine3x4 is null) throw new ArgumentNullException(nameof(affine3x4));
    if (affine3x4.Rows != 3 || affine3x4.Cols != 4)
        throw new ArgumentException("Expected a 3x4 affine matrix.", nameof(affine3x4));

    using var T = affine3x4.Type() == MatType.CV_64F ? affine3x4 : affine3x4.Clone();
    if (T.Type() != MatType.CV_64F) T.ConvertTo(T, MatType.CV_64F);

    // A = upper-left 3x3, t = last column
    using var A = new Mat(T, new Rect(0, 0, 3, 3));
    double tx = T.At<double>(0, 3), ty = T.At<double>(1, 3), tz = T.At<double>(2, 3);

    // R = closest rotation to A (polar decomposition via SVD): A = U * W * Vt  => R = U * Vt
    using var W = new Mat();
    using var U = new Mat();
    using var Vt = new Mat();
    Cv2.SVDecomp(A, W, U, Vt);

    using var R = U * Vt;

    // Enforce proper rotation (no reflection): det(R) must be +1
    if (Cv2.Determinant(R) < 0)
    {
        // Flip sign of last column of U, then recompute R
        U.Col(2).ConvertTo(U.Col(2), U.Col(2).Type(), -1.0); // U[:,2] *= -1
        R.Dispose();
        using var R2 = U * Vt;

        double x2 = R2.At<double>(0, 0) * p.X + R2.At<double>(0, 1) * p.Y + R2.At<double>(0, 2) * p.Z + tx;
        double y2 = R2.At<double>(1, 0) * p.X + R2.At<double>(1, 1) * p.Y + R2.At<double>(1, 2) * p.Z + ty;
        double z2 = R2.At<double>(2, 0) * p.X + R2.At<double>(2, 1) * p.Y + R2.At<double>(2, 2) * p.Z + tz;
        return new ScanPointXYZ(x2, y2, z2);
    }

    double x = R.At<double>(0, 0) * p.X + R.At<double>(0, 1) * p.Y + R.At<double>(0, 2) * p.Z + tx;
    double y = R.At<double>(1, 0) * p.X + R.At<double>(1, 1) * p.Y + R.At<double>(1, 2) * p.Z + ty;
    double z = R.At<double>(2, 0) * p.X + R.At<double>(2, 1) * p.Y + R.At<double>(2, 2) * p.Z + tz;
    return new ScanPointXYZ(x, y, z);
}