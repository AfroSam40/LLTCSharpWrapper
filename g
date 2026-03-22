public struct StichParams
{
    public PointCloud cloud;
    public List<RectangleF> fidCropRegions;
    public PlaneFitParams planeFitParams;
}

public static (Transform? transform, List<ITuple> src, List<ITuple> dst) GetStichingTransform(StichParams src, StichParams dst)
{
    // Validate inputs
    if (src.fidCropRegions == null || dst.fidCropRegions == null ||
        src.fidCropRegions.Count == 0 || dst.fidCropRegions.Count == 0 ||
        src.fidCropRegions.Count != dst.fidCropRegions.Count)
        return (null, null, null);

    var srcResults = new List<ITuple>();
    var dstResults = new List<ITuple>();
    var srcCorners = new List<Vector3>();
    var dstCorners = new List<Vector3>();

    // Collect src corners
    foreach (var r in src.fidCropRegions)
    {
        var crop = ClipToRectangleXY(src.cloud, r);
        src.planeFitParams.cropWidth = r.Width;
        src.planeFitParams.cropHeight = r.Height;

        var res = FindFiducial(crop, src.planeFitParams, false);
        srcResults.Add(res);

        if (res.Corners == null || res.Corners.Count == 0)
            return (null, null, null);

        srcCorners.AddRange(res.Corners);
    }

    // Collect dst corners
    foreach (var r in dst.fidCropRegions)
    {
        var crop = ClipToRectangleXY(dst.cloud, r);
        dst.planeFitParams.cropWidth = r.Width;
        dst.planeFitParams.cropHeight = r.Height;

        var res = FindFiducial(crop, dst.planeFitParams, false);
        dstResults.Add(res);

        if (res.Corners == null || res.Corners.Count == 0)
            return (null, null, null);

        dstCorners.AddRange(res.Corners);
    }

    // Fit transform
    if (srcCorners.Count == 0 || dstCorners.Count == 0 || srcCorners.Count != dstCorners.Count)
        return (null, null, null);

    return (new KabschUmeyama(srcCorners.ToArray(), dstCorners.ToArray()), srcResults, dstResults);
}