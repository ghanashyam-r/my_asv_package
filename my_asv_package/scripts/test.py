import rospy
import math
from geometry_msgs.msg import Twist, Vector3
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import PoseStamped, Vector3

pub = None
target_point = None
current_pose = None
target_angle_tolerance = math.radians(5)  # Tolerance angle to consider the robot has reached the target

def clbk_laser(msg):
    regions = {
        'right':  min(min(msg.ranges[0:143]), 10),
        'fright': min(min(msg.ranges[144:287]), 10),
        'front':  min(min(msg.ranges[288:431]), 10),
        'fleft':  min(min(msg.ranges[432:575]), 10),
        'left':   min(min(msg.ranges[576:719]), 10),
    }
   
    take_action(regions)
   
def take_action(regions):
    global target_point, current_pose

    if target_point is None or current_pose is None:
        return

    linear_y = 0  # Changed from linear_x to linear_y
    angular_z = 0
   
    state_description = ''

    distance_to_target = math.sqrt((target_point.x - current_pose.position.y) ** 2 + (target_point.y - current_pose.position.x) ** 2)  # Switched coordinates here
    angle_to_target = math.atan2(target_point.y - current_pose.position.x, target_point.x - current_pose.position.y)  # Switched coordinates here

    # Convert target angle to robot's frame of reference
    quaternion = (
        current_pose.orientation.x,
        current_pose.orientation.y,
        current_pose.orientation.z,
        current_pose.orientation.w
    )
    _, _, yaw = euler_from_quaternion(quaternion)

    # Calculate relative angle to the target
    relative_angle = angle_to_target - yaw

    if abs(relative_angle) > target_angle_tolerance:  # Rotate if not facing target
        state_description = 'Rotating to face target'
        linear_y = 0  # No linear motion during rotation
        angular_z = 0.5 if relative_angle > 0 else -0.5  # Rotate left if angle is positive, else right
    else:
        if distance_to_target > 0.1:  # If not reached target, move forward
            state_description = 'Moving towards target'
            linear_y = 0.3  # Move forward
            angular_z = 0
        else:
            state_description = 'Reached target'
            linear_y = 0
            angular_z = 0

    rospy.loginfo(state_description)
    rospy.loginfo("Distance to target: %.2f meters" % distance_to_target)  # Log distance to target
    
    msg = Twist()
    msg.linear = Vector3(0, linear_y, 0)
    msg.angular = Vector3(0, 0, angular_z)
    pub.publish(msg)

def clbk_odom(msg):
    global current_pose
    current_pose = msg.pose.pose

def main():
    global pub, target_point

    rospy.init_node('reading_laser')
   
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
   
    sub_laser = rospy.Subscriber('/m2wr/laser/scan', LaserScan, clbk_laser)
    sub_odom = rospy.Subscriber('/odom', Odometry, clbk_odom)

    target_point = Vector3(3.0, 3.0, 0.0)  # Set target point

    rospy.spin()

if __name__ == '__main__':
    main()
