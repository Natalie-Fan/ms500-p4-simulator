"""Upload-area drawing rules and colours for the GUI (the camera web page's orange box)."""
from __future__ import annotations

UPLOAD_COLOR = "#ff9500"
INVALID_COLOR = "#ff3b30"
FRAME_W, FRAME_H = 1920, 1080


def rect_problem(x: float, y: float, w: float, h: float, min_side: int, max_pixels: int) -> str | None:
    """Why a drawn rectangle breaks the firmware rules (flood_upload_rect_valid), or None."""
    if w < min_side or h < min_side:
        return f"Each side must be at least {min_side} px"
    if x < 0 or y < 0 or x + w > FRAME_W or y + h > FRAME_H:
        return "The area must lie inside the 1920x1080 frame"
    if w * h > max_pixels:
        return f"The area must cover at most {max_pixels} px"
    return None
