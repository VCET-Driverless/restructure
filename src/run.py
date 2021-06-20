
from perception.perception import get_object_estimates
from localization.motion_estimate.velocity_estimate import get_velocity
from localization.loclaization_mapping.slam import slam
from planning.path_plan import path_planning
from control.py_control import get_steering_angle, send

from system_manager.fault_manager.detect import check_fault
from system_manager.loger.logging import log

def main():
    while True:
        # perception
        frame = None
        ground_points = get_object_estimates(frame)
        
        # state estimate
        velocity = get_velocity()
        
        # localization and mapping
        mapp, location_in_map = slam(ground_points, velocity)
        
        # path planning
        path = path_planning(mapp, location_in_map)
        
        # control
        steering_angle = get_steering_angle(path, velocity)
        send(steering_angle)

        # system management
        check_fault()
        log()
main()