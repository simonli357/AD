import cv2
import numpy as np
import os

def rotate_image_and_mask(image, angle_deg, roi_mask=None):
    (h, w) = image.shape[:2]
    center = (w / 2, h / 2)
    rot_mat = cv2.getRotationMatrix2D(center, angle_deg, 1.0)

    # New size
    cos = np.abs(rot_mat[0, 0])
    sin = np.abs(rot_mat[0, 1])
    new_w = int((h * sin) + (w * cos))
    new_h = int((h * cos) + (w * sin))

    # Translate to center
    rot_mat[0, 2] += (new_w / 2) - center[0]
    rot_mat[1, 2] += (new_h / 2) - center[1]

    # Rotate image
    rotated_img = cv2.warpAffine(image, rot_mat, (new_w, new_h), flags=cv2.INTER_NEAREST, borderValue=0)

    # Create full-white mask same size as original image
    if roi_mask is None:
        white_mask = np.ones_like(image, dtype=np.uint8) * 255
    else:
        white_mask = roi_mask.copy()
    rotated_mask = cv2.warpAffine(white_mask, rot_mat, (new_w, new_h), flags=cv2.INTER_NEAREST, borderValue=0)

    return rotated_img, rotated_mask

def template_match(map_image, bev_image, mask=None, approx_position=None, search_radius=20, method=cv2.TM_CCOEFF_NORMED, use_full_map=False):
    if use_full_map or approx_position is None:
        roi = map_image
        roi_mask = mask
        offset_x = 0
        offset_y = 0
    else:
        h, w = bev_image.shape[:2]
        x, y = approx_position
        x1 = max(0, x - search_radius)
        y1 = max(0, y - search_radius)
        x2 = min(map_image.shape[1], x + w + search_radius)
        y2 = min(map_image.shape[0], y + h + search_radius)
        roi = map_image[y1:y2, x1:x2]
        if mask is not None:
            roi_mask = mask[y1:y2, x1:x2]
        else:
            roi_mask = None
        offset_x = x1
        offset_y = y1

    # Apply template matching, optionally with a mask
    if mask is not None and method in [cv2.TM_CCORR_NORMED, cv2.TM_SQDIFF]:
        result = cv2.matchTemplate(roi, bev_image, method, mask=mask)
    else:
        result = cv2.matchTemplate(roi, bev_image, method)

    min_val, max_val, min_loc, max_loc = cv2.minMaxLoc(result)

    if method in [cv2.TM_SQDIFF, cv2.TM_SQDIFF_NORMED]:
        match_loc = min_loc
        match_val = min_val
    else:
        match_loc = max_loc
        match_val = max_val

    best_x = offset_x + match_loc[0]
    best_y = offset_y + match_loc[1]

    return (best_x, best_y), match_val

# --- Load and resize images ---
current_dir = os.path.dirname(os.path.abspath(__file__))

# Map
map_img = cv2.imread(os.path.join(current_dir, "Track.png"), cv2.IMREAD_GRAYSCALE)
width_m = 20.696
height_m = 13.786
desired_ppm = 50
desired_width_px = int(width_m * desired_ppm)
desired_height_px = int(height_m * desired_ppm)
map_img = cv2.resize(map_img, (desired_width_px, desired_height_px), interpolation=cv2.INTER_NEAREST)

# BEV image
bev_img = cv2.imread(os.path.join(current_dir, "bevtest6.png"), cv2.IMREAD_GRAYSCALE)
bev_res = 200  # pixels per meter
bev_img = cv2.resize(
    bev_img,
    (int(bev_img.shape[1] * desired_ppm / bev_res), int(bev_img.shape[0] * desired_ppm / bev_res)),
    interpolation=cv2.INTER_NEAREST
)

yaw_angle_deg = 90
rot_bev, rot_mask = rotate_image_and_mask(bev_img, yaw_angle_deg)

methods = [
    cv2.TM_CCOEFF_NORMED, # doesnt accept roi mask
    cv2.TM_CCORR_NORMED,
    cv2.TM_SQDIFF,
    cv2.TM_SQDIFF_NORMED, # doesnt accept roi mask
]

for method in methods:
    print("Method:", method)
    # Pass mask only if method supports it
    mask_to_use = rot_mask if method in [cv2.TM_CCORR_NORMED, cv2.TM_SQDIFF] else None
    best_pos, score = template_match(map_img, rot_bev, mask=mask_to_use, use_full_map=True, method=method)
    print("Best match at:", best_pos, "Score:", score)

    h, w = rot_bev.shape
    map_img_copy = map_img.copy()
    cv2.rectangle(map_img_copy, best_pos, (best_pos[0]+w, best_pos[1]+h), 128, 2)
    cv2.imshow("Match", map_img_copy)
    cv2.imshow("BEV Rotated", rot_bev)
    cv2.imshow("Mask", rot_mask)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
