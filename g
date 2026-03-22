public Transform Inverted()
{
    var v = values;

    // Remove uniform scale from rotation part first
    double s = Math.Abs(v.s) < 1e-12 ? 1.0 : v.s;
    double invS = 1.0 / s;

    // For rigid transform: R^-1 = R^T
    double ir00 = v.r00, ir01 = v.r10, ir02 = v.r20;
    double ir10 = v.r01, ir11 = v.r11, ir12 = v.r21;
    double ir20 = v.r02, ir21 = v.r12, ir22 = v.r22;

    // Since p' = s*(R p) + t,
    // inverse is p = (1/s) * R^T * (p' - t)
    // so inverse translation is -(1/s) * R^T * t
    double itx = -invS * (ir00 * v.tx + ir01 * v.ty + ir02 * v.tz);
    double ity = -invS * (ir10 * v.tx + ir11 * v.ty + ir12 * v.tz);
    double itz = -invS * (ir20 * v.tx + ir21 * v.ty + ir22 * v.tz);

    return new Transform(
        (
            ir00, ir01, ir02,
            ir10, ir11, ir12,
            ir20, ir21, ir22,
            itx,  ity,  itz,
            invS
        )
    );
}