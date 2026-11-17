"""Camera/base transforms and right-handed palm-to-rubber-hand alignment.

Human palm +Z faces outward. Robot target +Z faces its opposite direction.
Tangent axis convention and wrist-to-hand-centre calibration remain pending.
"""

def palm_to_robot_target(palm_pose, camera_to_base, wrist_to_hand_center):
    raise NotImplementedError('Hand-centre calibration and axis convention are pending')
