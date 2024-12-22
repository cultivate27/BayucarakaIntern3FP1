import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Point

pointer_position = 640  # Dari tengah layar webcam (1280x720)

def callback(data):
    global pointer_position
    x_obj = data.x

    delta = x_obj - pointer_position

    tolerance = 1
    if abs(delta) == tolerance:
        motor_command = "S"
    elif delta > 0:
        motor_command = "R"
        pointer_position += 1
    else:
        motor_command = "L"
        pointer_position -= 1

    motor_pub.publish(motor_command)

rospy.init_node('control_node')
motor_pub = rospy.Publisher('motor_command', String, queue_size=10)
rospy.Subscriber('/detected_object_coordinates', Point, callback)

rospy.spin()