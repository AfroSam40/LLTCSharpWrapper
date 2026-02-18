// NuGet: Beckhoff.TwinCAT.Ads, Beckhoff.TwinCAT.Ads.TypeSystem
using System;
using System.Collections.Generic;
using TwinCAT.Ads;
using TwinCAT.Ads.TypeSystem;

static class AdsSearch
{
    // Returns full instance paths (e.g. "MAIN.fbAxis.Status.ActPos") that contain "term" (case-insensitive)
    public static List<string> Find(TcAdsClient c, string term, int max = 2000)
    {
        var l = SymbolLoaderFactory.Create(c, SymbolLoaderSettings.Default);
        var r = new List<string>(64);
        var t = term.AsSpan();
        void W(ISymbol s){ if (r.Count>=max) return; if (s.InstancePath?.Contains(term, StringComparison.OrdinalIgnoreCase)==true) r.Add(s.InstancePath);
            foreach (var x in s.SubSymbols) W(x); }
        foreach (var s in l.Symbols) W(s);
        return r;
    }
}