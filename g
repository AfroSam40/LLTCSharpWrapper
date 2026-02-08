var fx = Viewport.EffectsManager ?? new HelixToolkit.Wpf.SharpDX.DefaultEffectsManager();
Viewport.EffectsManager = fx;

var props = fx.GetType().GetProperties()
    .Select(p => p.Name)
    .Where(n => n.IndexOf("msaa", StringComparison.OrdinalIgnoreCase) >= 0
             || n.IndexOf("aa", StringComparison.OrdinalIgnoreCase) >= 0
             || n.IndexOf("fxaa", StringComparison.OrdinalIgnoreCase) >= 0
             || n.IndexOf("sample", StringComparison.OrdinalIgnoreCase) >= 0)
    .OrderBy(n => n);

System.Diagnostics.Debug.WriteLine("EffectsManager AA-related props:");
foreach (var p in props) System.Diagnostics.Debug.WriteLine("  " + p);