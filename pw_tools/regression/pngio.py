"""Minimal, dependency-free PNG I/O for 8-bit grayscale images.

Why hand-rolled: the regression harness must run on a bare `python3` with no
numpy / Pillow / cv2. EuRoC ships `cam0/data/*.png` as 8-bit grayscale,
non-interlaced, which is exactly the subset implemented here. Colour and 16-bit
PNGs are down-converted so that a foreign dataset does not silently fail.

Only the standard library (`zlib`, `struct`) is used.
"""

import struct
import zlib

PNG_MAGIC = b"\x89PNG\r\n\x1a\n"


class PngError(Exception):
    pass


def _chunk(tag, payload):
    return (
        struct.pack(">I", len(payload))
        + tag
        + payload
        + struct.pack(">I", zlib.crc32(tag + payload) & 0xFFFFFFFF)
    )


def encode_gray8(width, height, data, level=6):
    """Encode a tightly packed 8-bit grayscale buffer into PNG bytes."""
    if width <= 0 or height <= 0:
        raise PngError("bad size %dx%d" % (width, height))
    if len(data) != width * height:
        raise PngError(
            "buffer is %d bytes, expected %d" % (len(data), width * height)
        )
    raw = bytearray()
    mv = memoryview(bytes(data))
    for y in range(height):
        raw.append(0)  # filter type 0 (None)
        raw += mv[y * width : (y + 1) * width]
    ihdr = struct.pack(">IIBBBBB", width, height, 8, 0, 0, 0, 0)
    return (
        PNG_MAGIC
        + _chunk(b"IHDR", ihdr)
        + _chunk(b"IDAT", zlib.compress(bytes(raw), level))
        + _chunk(b"IEND", b"")
    )


def _unfilter(raw, width, height, bpp):
    stride = width * bpp
    out = bytearray(stride * height)
    pos = 0
    prev = bytearray(stride)
    for y in range(height):
        ft = raw[pos]
        pos += 1
        line = bytearray(raw[pos : pos + stride])
        pos += stride
        if len(line) != stride:
            raise PngError("truncated scanline %d" % y)
        if ft == 0:
            pass
        elif ft == 1:
            for i in range(bpp, stride):
                line[i] = (line[i] + line[i - bpp]) & 0xFF
        elif ft == 2:
            for i in range(stride):
                line[i] = (line[i] + prev[i]) & 0xFF
        elif ft == 3:
            for i in range(stride):
                a = line[i - bpp] if i >= bpp else 0
                line[i] = (line[i] + ((a + prev[i]) >> 1)) & 0xFF
        elif ft == 4:
            for i in range(stride):
                a = line[i - bpp] if i >= bpp else 0
                b = prev[i]
                c = prev[i - bpp] if i >= bpp else 0
                p = a + b - c
                pa, pb, pc = abs(p - a), abs(p - b), abs(p - c)
                if pa <= pb and pa <= pc:
                    pr = a
                elif pb <= pc:
                    pr = b
                else:
                    pr = c
                line[i] = (line[i] + pr) & 0xFF
        else:
            raise PngError("unknown filter type %d on row %d" % (ft, y))
        out[y * stride : (y + 1) * stride] = line
        prev = line
    return out


def decode_gray8(blob):
    """Decode PNG bytes -> (width, height, bytes) tightly packed 8-bit gray."""
    if blob[:8] != PNG_MAGIC:
        raise PngError("not a PNG")
    pos = 8
    width = height = depth = color = None
    interlace = 0
    idat = []
    while pos + 8 <= len(blob):
        (length,) = struct.unpack(">I", blob[pos : pos + 4])
        tag = blob[pos + 4 : pos + 8]
        payload = blob[pos + 8 : pos + 8 + length]
        pos += 12 + length
        if tag == b"IHDR":
            width, height, depth, color, _comp, _filt, interlace = struct.unpack(
                ">IIBBBBB", payload
            )
        elif tag == b"IDAT":
            idat.append(payload)
        elif tag == b"IEND":
            break
    if width is None:
        raise PngError("no IHDR")
    if interlace != 0:
        raise PngError("interlaced PNG not supported")
    if depth not in (8, 16):
        raise PngError("bit depth %d not supported" % depth)
    if color not in (0, 2, 4, 6):
        raise PngError("colour type %d not supported (no palette support)" % color)

    samples = {0: 1, 2: 3, 4: 2, 6: 4}[color]
    bpp = samples * (depth // 8)
    raw = _unfilter(zlib.decompress(b"".join(idat)), width, height, bpp)

    if color == 0 and depth == 8:
        return width, height, bytes(raw)

    out = bytearray(width * height)
    step = depth // 8
    for i in range(width * height):
        base = i * bpp
        if color in (0, 4):
            v = raw[base]
        else:
            r = raw[base]
            g = raw[base + step]
            b = raw[base + 2 * step]
            v = (r * 299 + g * 587 + b * 114) // 1000
        out[i] = v
    return width, height, bytes(out)
