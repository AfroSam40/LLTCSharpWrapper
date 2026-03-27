using System;
using System.IO;

public static class Logger
{
    private static readonly string LogFilePath =
        Path.Combine(AppContext.BaseDirectory, "app.log");

    public static void Log(string message)
    {
        string line = $"[{DateTime.Now:yyyy-MM-dd HH:mm:ss}] {message}";
        File.AppendAllText(LogFilePath, line + Environment.NewLine);
    }

    public static void LogError(string message, Exception ex)
    {
        string line = $"[{DateTime.Now:yyyy-MM-dd HH:mm:ss}] ERROR: {message}{Environment.NewLine}{ex}";
        File.AppendAllText(LogFilePath, line + Environment.NewLine + Environment.NewLine);
    }
}