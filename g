using System;
using System.Numerics;

public static Transform WorkToWorldFromFiducial(ReadOnlySpan<Vector3> cornersTLTRBRBL, int originIndex)
{
    if (cornersTLTRBRBL.Length != 4) throw new ArgumentException("Expected 4 corners TL,TR,BR,BL.");
    if ((uint)originIndex > 3) throw new ArgumentOutOfRangeException(nameof(originIndex));

    var c = cornersTLTRBRBL;
    var origin = c[originIndex];

    var top    = c[1] - c[0];
    var bottom = c[2] - c[3];
    var left   = c[3] - c[0];
    var right  = c[2] - c[1];

    var x = Vector3.Normalize(top + bottom);
    var y0 = Vector3.Normalize(left + right);

    var z = Vector3.Normalize(Vector3.Cross(x, y0));
    var y = Vector3.Normalize(Vector3.Cross(z, x));

    // Optional: enforce your convention here by flipping x/y if needed
    // x = -x;  // if you want +X to point left instead of right
    // y = -y;  // if you want +Y to point up instead of down

    // Work -> World matrix (axes in columns, translation is originWorld)
    var M = new Matrix4x4(
        x.X, y.X, z.X, 0f,
        x.Y, y.Y, z.Y, 0f,
        x.Z, y.Z, z.Z, 0f,
        origin.X, origin.Y, origin.Z, 1f
    );

    return new Transform { transformMatrix = M };
}