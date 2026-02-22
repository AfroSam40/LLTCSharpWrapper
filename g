public static ScanPointXYZ Transform3DPointFast(
    in ScanPointXYZ p,
    in (double r00, double r01, double r02,
        double r10, double r11, double r12,
        double r20, double r21, double r22,
        double tx,  double ty,  double tz) rt)
    => new ScanPointXYZ(
        rt.r00 * p.X + rt.r01 * p.Y + rt.r02 * p.Z + rt.tx,
        rt.r10 * p.X + rt.r11 * p.Y + rt.r12 * p.Z + rt.ty,
        rt.r20 * p.X + rt.r21 * p.Y + rt.r22 * p.Z + rt.tz
    );
```0