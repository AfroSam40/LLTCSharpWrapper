public static ScanPointXYZ Transform3DPointFast_EulerZYX(
    in ScanPointXYZ p,
    in (double rx, double ry, double rz, double tx, double ty, double tz) rt)
{
    // angles in radians
    double cx = Math.Cos(rt.rx), sx = Math.Sin(rt.rx);
    double cy = Math.Cos(rt.ry), sy = Math.Sin(rt.ry);
    double cz = Math.Cos(rt.rz), sz = Math.Sin(rt.rz);

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

    return new ScanPointXYZ(
        r00 * p.X + r01 * p.Y + r02 * p.Z + rt.tx,
        r10 * p.X + r11 * p.Y + r12 * p.Z + rt.ty,
        r20 * p.X + r21 * p.Y + r22 * p.Z + rt.tz
    );
}