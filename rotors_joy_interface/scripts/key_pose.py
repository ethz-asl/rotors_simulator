#!/usr/bin/env python3
# Author: Nadia Khodaei
import rospy
from geometry_msgs.msg import PoseStamped
import sys, select, termios, tty
import tf.transformations

msg = """
Reading from the keyboard and Publishing to PoseStamped!
---------------------------
Moving around:
   Arrow Keys:
   ↑    ↓    ←    →

↑/↓ : move forward/backward in x direction
←/→ : rotate left/right (yaw) in z direction
i/, : move up/down in z direction
k/m : move left/right in y direction
anything else : stop

Adjust speed:
   + : increase step
   - : decrease step

s : reset position to default

CTRL-C to quit
"""

# Arrow key codes
arrowBindings = {
    '\x1b[A': (1, 0, 0, 0),  # up arrow
    '\x1b[B': (-1, 0, 0, 0),  # down arrow
    '\x1b[D': (0, 0, 0, 1),  # left arrow (rotate left)
    '\x1b[C': (0, 0, 0, -1),  # right arrow (rotate right)
}

# Normal key codes
moveBindings = {
    'i': (0, 0, 1, 0),
    ',': (0, 0, -1, 0),
    'k': (0, 1, 0, 0),
    'm': (0, -1, 0, 0),
    's': 'reset',
    '+': 'increase_step',
    '-': 'decrease_step'
}

def getKey():
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
        if key == '\x1b':
            key += sys.stdin.read(2)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        return key
    else:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        return ''

if __name__ == "__main__":
    settings = termios.tcgetattr(sys.stdin)

    rospy.init_node('keyboard_teleop_position')
    pub = rospy.Publisher('/firefly/command/pose', PoseStamped, queue_size=10)

    # Default position values
    default_x = 0
    default_y = 0
    default_z = 1.0
    default_yaw = 0
    x = default_x
    y = default_y
    z = default_z
    yaw = default_yaw
    step = 0.1  # Initial step value, now a float

    try:
        print(msg)
        while not rospy.is_shutdown():
            key = getKey()
            if key in arrowBindings.keys():
                x += arrowBindings[key][0] * step
                y += arrowBindings[key][1] * step
                z += arrowBindings[key][2] * step
                yaw += arrowBindings[key][3] * step
            elif key in moveBindings.keys():
                if moveBindings[key] == 'reset':
                    x = default_x
                    y = default_y
                    z = default_z
                    yaw = default_yaw
                    print("Position reset to default")
                elif moveBindings[key] == 'increase_step':
                    step = min(step + 0.1, 10)  # Increase step, max is 10
                    print(f"Step increased to: {step}")
                elif moveBindings[key] == 'decrease_step':
                    step = max(step - 0.1, 0.1)  # Decrease step, min is 0.1
                    print(f"Step decreased to: {step}")
                else:
                    x += moveBindings[key][0] * step
                    y += moveBindings[key][1] * step
                    z += moveBindings[key][2] * step
                    yaw += moveBindings[key][3] * step
            else:
                if key == '\x03':
                    break

            pose = PoseStamped()
            pose.header.stamp = rospy.Time.now()
            pose.header.frame_id = 'world'
            pose.pose.position.x = x 
            pose.pose.position.y = y 
            pose.pose.position.z = z 

            # Convert yaw to quaternion
            q = tf.transformations.quaternion_from_euler(0, 0, yaw)
            pose.pose.orientation.x = q[0]
            pose.pose.orientation.y = q[1]
            pose.pose.orientation.z = q[2]
            pose.pose.orientation.w = q[3]

            pub.publish(pose)

    except Exception as e:
        print(e)

    finally:
        pose = PoseStamped()
        pose.header.stamp = rospy.Time.now()
        pose.header.frame_id = 'world'
        pose.pose.position.x = 0
        pose.pose.position.y = 0
        pose.pose.position.z = 1.0
        pose.pose.orientation.w = 1.0
        pub.publish(pose)

        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

