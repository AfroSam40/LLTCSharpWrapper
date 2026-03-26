var closest = points
    .OrderBy(p => Math.Abs(p.Z - target.Z))
    .ThenBy(p =>
        (p.X - target.X) * (p.X - target.X) +
        (p.Y - target.Y) * (p.Y - target.Y) +
        (p.Z - target.Z) * (p.Z - target.Z))
    .FirstOrDefault();