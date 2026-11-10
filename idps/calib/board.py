"""A printable checkerboard for the lens calibration (``idps-calib board``)."""
from __future__ import annotations

from pathlib import Path


def board_svg(cols: int = 10, rows: int = 7, square_mm: float = 25.0, margin_mm: float = 10.0) -> str:
    """An SVG of ``cols`` x ``rows`` squares (so cols-1 x rows-1 inner corners), true size in mm.
    10 x 7 squares of 25 mm fit an A4 sheet in landscape."""
    w, h = cols * square_mm + 2 * margin_mm, rows * square_mm + 2 * margin_mm
    out = [f'<svg xmlns="http://www.w3.org/2000/svg" width="{w}mm" height="{h}mm" viewBox="0 0 {w} {h}">',
           f'<rect width="{w}" height="{h}" fill="white"/>']
    for r in range(rows):
        for c in range(cols):
            if (r + c) % 2 == 0:
                out.append(f'<rect x="{margin_mm + c * square_mm}" y="{margin_mm + r * square_mm}" '
                           f'width="{square_mm}" height="{square_mm}" fill="black"/>')
    out.append(f'<text x="{margin_mm}" y="{h - 3}" font-size="3.5" font-family="sans-serif">IDPS lens board: '
               f'{cols} x {rows} squares of {square_mm:g} mm. Print at 100 % (no scaling) and measure a square.</text>')
    out.append("</svg>")
    return "\n".join(out)


def write_board(path, cols: int = 10, rows: int = 7, square_mm: float = 25.0) -> Path:
    path = Path(path)
    path.write_text(board_svg(cols, rows, square_mm))
    return path
