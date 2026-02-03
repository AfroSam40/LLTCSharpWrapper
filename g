using System;

public static class EndianHex
{
    /// <summary>
    /// Converts a double to hex bytes in little-endian order.
    /// Example output: "9A 99 99 99 99 99 F1 3F"
    /// </summary>
    public static string DoubleToLittleEndianHex(double value, bool spaced = true)
    {
        byte[] bytes = BitConverter.GetBytes(value); // little-endian on Windows/x86/x64

        // If you ever run on big-endian (rare), enforce little:
        if (!BitConverter.IsLittleEndian)
            Array.Reverse(bytes);

        return spaced
            ? BitConverter.ToString(bytes).Replace("-", " ")
            : BitConverter.ToString(bytes).Replace("-", "");
    }
}