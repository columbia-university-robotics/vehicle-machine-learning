import rospy
from std_msgs.msg import Float64
from ros_service.srv import ExcavatorArmRotationSrv

class ExcavatorArmRotationService():
    def __init__(self):
        self.move_mount_joint = rospy.Publisher('/excavator_1/mount_joint_controller/command', Float64, queue_size=1)
        self.srv = rospy.Service("/excavator_1/services/arm_rotation" , ExcavatorArmRotationSrv , self.arm_rotation_callback)

    def arm_rotation_callback(self, request):
        pass