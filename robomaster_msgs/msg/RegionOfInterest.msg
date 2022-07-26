# A region of interest (ROS) in an image
# All values are in [0, 1] and are relative to image width and height

float32 x_offset  # leftmost pixel of the ROI
                  # (0 if the ROI includes the left edge of the image)
float32 y_offset  # topmost pixel of the ROI
                  # (0 if the ROI includes the top edge of the image)
float32 height    # height of ROI
float32 width     # width of ROI
