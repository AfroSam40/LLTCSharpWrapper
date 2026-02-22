using System.Numerics;

public static pointmatcher.net.EuclideanTransform MakeInitialGuess(
    (double r00, double r01, double r02,
     double r10, double r11, double r12,
     double r20, double r21, double r22,
     double tx,  double ty,  double tz) rt)
{
    // Build a Matrix4x4 from rotation only (translation handled separately)
    var m = new Matrix4x4(
        (float)rt.r00, (float)rt.r01, (float)rt.r02, 0f,
        (float)rt.r10, (float)rt.r11, (float)rt.r12, 0f,
        (float)rt.r20, (float)rt.r21, (float)rt.r22, 0f,
        0f,            0f,            0f,            1f);

    return new pointmatcher.net.EuclideanTransform
    {
        rotation = Quaternion.CreateFromRotationMatrix(m),
        translation = new Vector3((float)rt.tx, (float)rt.ty, (float)rt.tz)
    };
}