public static ScanPointXYZ Transform3DPointFast_FromGetTransform(
    in ScanPointXYZ p,
    in (double Rx, double Ry, double Rz,
        double Tx, double Ty, double Tz,
        double Sx, double Sy, double Sz,
        double ShXY, double ShXZ, double ShYZ,
        string Debug) t)
{
    // Assumes Rx,Ry,Rz are in radians and rotation order is Rz * Ry * Rx (ZYX)
    double cx = Math.Cos(t.Rx), sx = Math.Sin(t.Rx);
    double cy = Math.Cos(t.Ry), sy = Math.Sin(t.Ry);
    double cz = Math.Cos(t.Rz), sz = Math.Sin(t.Rz);

    // R = Rz * Ry * Rx
    double r00 = cz * cy;
    double r01 = cz * sy * sx - sz * cx;
    double r02 = cz * sy * cx + sz * sx;

    double r10 = sz * cy;
    double r11 = sz * sy * sx + cz * cx;
    double r12 = sz * sy * cx - cz * sx;

    double r20 = -sy;
    double r21 = cy * sx;
    double r22 = cy * cx;

    return new ScanPointXYZ(
        r00 * p.X + r01 * p.Y + r02 * p.Z + t.Tx,
        r10 * p.X + r11 * p.Y + r12 * p.Z + t.Ty,
        r20 * p.X + r21 * p.Y + r22 * p.Z + t.Tz
    );
}