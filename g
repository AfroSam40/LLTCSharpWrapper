import os
import struct
from typing import List, Tuple

def load_point_cloud(file_path: str) -> List[Tuple[float, float, float]]:
    if not file_path or not str(file_path).strip():
        raise ValueError("file_path is null/empty/whitespace")

    if not os.path.isfile(file_path):
        raise FileNotFoundError(file_path)

    points: List[Tuple[float, float, float]] = []

    with open(file_path, "rb") as f:
        # C# BinaryReader.ReadInt32() => 4 bytes (little-endian on Windows/.NET)
        raw = f.read(4)
        if len(raw) != 4:
            raise EOFError("File too short: missing point count (int32)")
        (count,) = struct.unpack("<i", raw)

        if count < 0:
            raise ValueError(f"Invalid point count: {count}")

        # Each point = 3 doubles = 24 bytes
        bytes_needed = count * 24
        data = f.read(bytes_needed)
        if len(data) != bytes_needed:
            raise EOFError(f"File too short: expected {bytes_needed} bytes of point data, got {len(data)}")

        # Unpack all doubles at once (faster than looping reads)
        doubles = struct.unpack("<" + "d" * (count * 3), data)

        # Group into (x,y,z)
        points = [(doubles[i], doubles[i+1], doubles[i+2]) for i in range(0, len(doubles), 3)]

    return points