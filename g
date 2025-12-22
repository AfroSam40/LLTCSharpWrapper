using OpenCvSharp;

public static class Affine3DApply
{
    /// <summary>
    /// Applies a 3x4 affine transform (from Cv2.EstimateAffine3D) to a 3D point.
    /// </summary>
    public static Point3d TransformPoint(Point3d p, Mat affine3x4)
    {
        if (affine3x4 == null) throw new ArgumentNullException(nameof(affine3x4));
        if (affine3x4.Rows != 3 || affine3x4.Cols != 4)
            throw new ArgumentException("Expected a 3x4 affine matrix.", nameof(affine3x4));

        // Ensure we read as double
        double a00 = affine3x4.Get<double>(0, 0), a01 = affine3x4.Get<double>(0, 1),
               a02 = affine3x4.Get<double>(0, 2), a03 = affine3x4.Get<double>(0, 3);

        double a10 = affine3x4.Get<double>(1, 0), a11 = affine3x4.Get<double>(1, 1),
               a12 = affine3x4.Get<double>(1, 2), a13 = affine3x4.Get<double>(1, 3);

        double a20 = affine3x4.Get<double>(2, 0), a21 = affine3x4.Get<double>(2, 1),
               a22 = affine3x4.Get<double>(2, 2), a23 = affine3x4.Get<double>(2, 3);

        double x = p.X, y = p.Y, z = p.Z;

        double xp = a00 * x + a01 * y + a02 * z + a03;
        double yp = a10 * x + a11 * y + a12 * z + a13;
        double zp = a20 * x + a21 * y + a22 * z + a23;

        return new Point3d(xp, yp, zp);
    }
}