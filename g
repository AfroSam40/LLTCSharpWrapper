void PrintTransform(string name, pointmatcher.net.EuclideanTransform T)
{
    var q = Quaternion.Normalize(T.rotation);
    var t = T.translation;
    System.Diagnostics.Debug.WriteLine(
        $"{name}: t=({t.X:F6},{t.Y:F6},{t.Z:F6})  q=({q.X:F6},{q.Y:F6},{q.Z:F6},{q.W:F6})");
}

PrintTransform("Init", initial);
PrintTransform("ICP ", refined);
PrintTransform("Δ   ", delta);

System.Diagnostics.Debug.WriteLine($"Δ translation |dT| = {dTMag:F6}");
System.Diagnostics.Debug.WriteLine($"Δ rotation    = {dAngleDeg:F6} deg");