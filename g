using MathNet.Numerics.LinearAlgebra;
// using pointmatcher.net;   // whatever the library namespace is in your project

static EuclideanTransform KabschToInitialTransform(
    (double r00, double r01, double r02,
     double r10, double r11, double r12,
     double r20, double r21, double r22,
     double tx,  double ty,  double tz) rt)
{
    var M = Matrix<double>.Build.DenseIdentity(4);
    M[0,0]=rt.r00; M[0,1]=rt.r01; M[0,2]=rt.r02; M[0,3]=rt.tx;
    M[1,0]=rt.r10; M[1,1]=rt.r11; M[1,2]=rt.r12; M[1,3]=rt.ty;
    M[2,0]=rt.r20; M[2,1]=rt.r21; M[2,2]=rt.r22; M[2,3]=rt.tz;

    return new EuclideanTransform(M); // (if your build uses a different ctor/factory, adjust)
}