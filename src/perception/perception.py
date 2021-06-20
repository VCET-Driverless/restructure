
def detection(frame):
    detections = None

    return detections

def inverse_perspective_mapping(detections):
    ground_points = None

    return ground_points

def get_object_estimates(frame):
    detections = detection(frame)
    ground_points = inverse_perspective_mapping(detections)

    return ground_points