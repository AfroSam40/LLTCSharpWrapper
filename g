using System;
using System.Collections.Generic;
using System.Drawing;
using System.Linq;

public static class DogboneHoleDetector
{
    /// <summary>
    /// Finds a rectangular/square "hole" with a mouse-ear (dogbone) on one corner
    /// and annotates the corner opposite to the ear.
    /// 
    /// Assumptions:
    /// - The bitmap is essentially binary or grayscale.
    /// - Holes are darker than the plate/background (e.g. 0 = hole, 255 = plate),
    ///   so we treat pixels with intensity &lt; holeThreshold as "hole".
    /// - The dogbone ear occurs at one of the 4 axis-aligned corners of the hole.
    /// </summary>
    /// <param name="input">Source bitmap (not modified).</param>
    /// <param name="annotatedCorner">
    /// Pixel coordinates of the corner opposite the mouse ear (in image space).
    /// PointF.Empty if no suitable hole was found.
    /// </param>
    /// <param name="holeThreshold">
    /// Intensity threshold to decide "hole" vs "non-hole" (0–255).
    /// </param>
    /// <param name="earCornerRatioThreshold">
    /// How asymmetric the corner distances to the centroid must be to count as a
    /// dogbone (typical 1.15–1.3). Larger means stricter dogbone detection.
    /// </param>
    /// <returns>
    /// A new bitmap with a red cross drawn at the opposite corner, or a simple
    /// copy of the input if no dogbone-like hole was found.
    /// </returns>
    public static Bitmap AnnotateDogboneOppositeCorner(
        Bitmap input,
        out PointF annotatedCorner,
        byte holeThreshold = 64,
        double earCornerRatioThreshold = 1.2)
    {
        if (input == null) throw new ArgumentNullException(nameof(input));

        int width = input.Width;
        int height = input.Height;

        // 1. Build a binary "hole" mask (true = hole pixel).
        bool[,] holeMask = new bool[width, height];

        for (int y = 0; y < height; y++)
        {
            for (int x = 0; x < width; x++)
            {
                Color c = input.GetPixel(x, y);
                // Simple grayscale luminance
                int intensity = (int)(0.299 * c.R + 0.587 * c.G + 0.114 * c.B);
                bool isHole = intensity < holeThreshold;
                holeMask[x, y] = isHole;
            }
        }

        // 2. Connected-component labeling on hole pixels
        bool[,] visited = new bool[width, height];
        var components = new List<Component>();

        for (int y = 0; y < height; y++)
        {
            for (int x = 0; x < width; x++)
            {
                if (!holeMask[x, y] || visited[x, y])
                    continue;

                var comp = FloodFillComponent(holeMask, visited, x, y, width, height);
                components.Add(comp);
            }
        }

        // 3. Find the component that best matches "square/rect with dogbone ear"
        Component bestDogbone = null;
        int bestOppCornerIndex = -1;
        double bestEarScore = 0.0;

        foreach (var comp in components)
        {
            // Ignore tiny components
            if (comp.Pixels.Count < 50)
                continue;

            int bw = comp.MaxX - comp.MinX + 1;
            int bh = comp.MaxY - comp.MinY + 1;
            if (bw < 5 || bh < 5)
                continue;

            double aspect = (double)bw / bh;
            if (aspect < 0.5 || aspect > 2.0)
                continue; // too skinny or too flat to be "square-ish / rectangular"

            // Compute centroid of the hole pixels
            double sumX = 0, sumY = 0;
            foreach (var p in comp.Pixels)
            {
                sumX += p.X;
                sumY += p.Y;
            }
            double cx = sumX / comp.Pixels.Count;
            double cy = sumY / comp.Pixels.Count;

            // corners of bounding box
            var corners = new[]
            {
                new PointF(comp.MinX, comp.MinY), // 0
                new PointF(comp.MaxX, comp.MinY), // 1
                new PointF(comp.MaxX, comp.MaxY), // 2
                new PointF(comp.MinX, comp.MaxY)  // 3
            };

            double[] dist2 = new double[4];
            for (int i = 0; i < 4; i++)
            {
                double dx = corners[i].X - cx;
                double dy = corners[i].Y - cy;
                dist2[i] = dx * dx + dy * dy;
            }

            double maxD2 = dist2.Max();
            double minD2 = dist2.Min();
            double ratio = maxD2 / Math.Max(minD2, 1e-6);

            // For a perfect rectangle with no ear, all four distances are similar,
            // so maxD2/minD2 ~ 1. For a dogbone shape the centroid is pulled
            // toward the ear corner, making the opposite corner significantly
            // farther and thus ratio > 1.
            if (ratio < earCornerRatioThreshold)
                continue;

            // Get index of farthest corner (this is opposite the ear)
            int farthestCornerIndex = 0;
            for (int i = 1; i < 4; i++)
            {
                if (dist2[i] > dist2[farthestCornerIndex])
                    farthestCornerIndex = i;
            }

            // Use "ratio" as a score; pick the most dogbone-looking component
            if (ratio > bestEarScore)
            {
                bestEarScore = ratio;
                bestDogbone = comp;
                bestOppCornerIndex = farthestCornerIndex;
            }
        }

        // 4. If nothing found, just return a copy
        var output = new Bitmap(input);
        annotatedCorner = PointF.Empty;

        if (bestDogbone == null || bestOppCornerIndex < 0)
            return output;

        // Compute final opposite-corner coordinates (pixel space)
        var bbCorners = new[]
        {
            new PointF(bestDogbone.MinX, bestDogbone.MinY),
            new PointF(bestDogbone.MaxX, bestDogbone.MinY),
            new PointF(bestDogbone.MaxX, bestDogbone.MaxY),
            new PointF(bestDogbone.MinX, bestDogbone.MaxY)
        };

        annotatedCorner = bbCorners[bestOppCornerIndex];

        // 5. Draw a red crosshair at that corner
        using (var g = Graphics.FromImage(output))
        {
            const int r = 6; // half-size of crosshair
            using (var pen = new Pen(Color.Red, 2))
            {
                float x = annotatedCorner.X;
                float y = annotatedCorner.Y;

                g.DrawEllipse(pen, x - r, y - r, 2 * r, 2 * r);
                g.DrawLine(pen, x - r, y, x + r, y);
                g.DrawLine(pen, x, y - r, x, y + r);
            }
        }

        return output;
    }

    // --- helpers -------------------------------------------------------------

    private class Component
    {
        public List<Point> Pixels { get; } = new List<Point>();
        public int MinX = int.MaxValue;
        public int MaxX = int.MinValue;
        public int MinY = int.MaxValue;
        public int MaxY = int.MinValue;
    }

    private static Component FloodFillComponent(
        bool[,] mask,
        bool[,] visited,
        int startX,
        int startY,
        int width,
        int height)
    {
        var comp = new Component();
        var queue = new Queue<Point>();
        queue.Enqueue(new Point(startX, startY));
        visited[startX, startY] = true;

        int[] dx = { 1, -1, 0, 0 };
        int[] dy = { 0, 0, 1, -1 }; // 4-connected; change to 8-connected if you want

        while (queue.Count > 0)
        {
            var p = queue.Dequeue();
            comp.Pixels.Add(p);

            if (p.X < comp.MinX) comp.MinX = p.X;
            if (p.X > comp.MaxX) comp.MaxX = p.X;
            if (p.Y < comp.MinY) comp.MinY = p.Y;
            if (p.Y > comp.MaxY) comp.MaxY = p.Y;

            for (int k = 0; k < 4; k++)
            {
                int nx = p.X + dx[k];
                int ny = p.Y + dy[k];
                if (nx < 0 || nx >= width || ny < 0 || ny >= height)
                    continue;
                if (visited[nx, ny]) continue;
                if (!mask[nx, ny]) continue;

                visited[nx, ny] = true;
                queue.Enqueue(new Point(nx, ny));
            }
        }

        return comp;
    }
}