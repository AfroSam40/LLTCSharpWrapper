using System;
using System.Diagnostics;
using System.Threading;
using System.Threading.Tasks;

public static class WaitUntil
{
    // Sync
    public static bool Till(Func<bool> condition, TimeSpan timeout, TimeSpan? pollInterval = null)
    {
        if (condition == null) throw new ArgumentNullException(nameof(condition));
        if (timeout < TimeSpan.Zero) throw new ArgumentOutOfRangeException(nameof(timeout));

        var poll = pollInterval ?? TimeSpan.FromMilliseconds(10);
        if (poll <= TimeSpan.Zero) poll = TimeSpan.FromMilliseconds(1);

        var sw = Stopwatch.StartNew();
        while (sw.Elapsed < timeout)
        {
            if (condition()) return true;
            Thread.Sleep(poll);
        }

        // One last check right at/after timeout
        return condition();
    }

    // Async (preferred if you're on UI thread / don't want to block)
    public static async Task<bool> TillAsync(
        Func<bool> condition,
        TimeSpan timeout,
        TimeSpan? pollInterval = null,
        CancellationToken ct = default)
    {
        if (condition == null) throw new ArgumentNullException(nameof(condition));
        if (timeout < TimeSpan.Zero) throw new ArgumentOutOfRangeException(nameof(timeout));

        var poll = pollInterval ?? TimeSpan.FromMilliseconds(10);
        if (poll <= TimeSpan.Zero) poll = TimeSpan.FromMilliseconds(1);

        var sw = Stopwatch.StartNew();
        while (sw.Elapsed < timeout)
        {
            ct.ThrowIfCancellationRequested();
            if (condition()) return true;
            await Task.Delay(poll, ct).ConfigureAwait(false);
        }

        return condition();
    }
}