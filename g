using System;
using System.Globalization;
using System.Linq;

public static class EndianHex
{
    /// <summary>
    /// Converts a little-endian hex byte string (8 bytes) into a double.
    /// Accepts formats like:
    /// "9A 99 99 99 99 99 F1 3F"
    /// "9A-99-99-99-99-99-F1-3F"
    /// "9A9999999999F13F"
    /// </summary>
    public static double LittleEndianHexToDouble(string hex)
    {
        if (hex == null) throw new ArgumentNullException(nameof(hex));

        // Keep only hex digits
        string clean = new string(hex.Where(Uri.IsHexDigit).ToArray());

        if (clean.Length != 16)
            throw new ArgumentException("Hex must represent exactly 8 bytes (16 hex chars).", nameof(hex));

        byte[] bytes = new byte[8];
        for (int i = 0; i < 8; i++)
        {
            string byteHex = clean.Substring(i * 2, 2);
            bytes[i] = byte.Parse(byteHex, NumberStyles.HexNumber, CultureInfo.InvariantCulture);
        }

        // Ensure bytes are interpreted as little-endian
        if (!BitConverter.IsLittleEndian)
            Array.Reverse(bytes);

        return BitConverter.ToDouble(bytes, 0);
    }
}