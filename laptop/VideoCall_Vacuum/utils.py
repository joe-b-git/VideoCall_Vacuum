import numpy as np

def calculate_center(box):
    """Calculates the center coordinates of a bounding box."""
    x, y, w, h = box
    center_x = x + w / 2
    center_y = y + h / 2
    return center_x, center_y

def calculate_bottom_center(box):
    """Calculates the bottom center coordinates of a bounding box."""
    x, y, w, h = box
    bottom_center_x = x + w / 2
    bottom_center_y = y + h
    return bottom_center_x, bottom_center_y
