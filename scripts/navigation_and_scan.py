#!/usr/bin/env python
import actionlib
from actionlib_msgs.msg import GoalID
from geometry_msgs.msg import Twist, PoseStamped
import math
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseActionResult
import numpy as np
import rospy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
import tf


def scan_callback(msg):
    global g_range_ahead
    tmp = [msg.ranges[0]]
    for i in range(1, 21):
        tmp.append(msg.ranges[i])
    for i in range(len(msg.ranges) - 21, len(msg.ranges)):
        tmp.append(msg.ranges[i])
    g_range_ahead = min(tmp)


def detect_qr(msg):
    global qr_detected
    if len(str(msg)) > 40:
        qr_detected = decode_msg(str(msg))
    else:
        qr_detected = None


def decode_msg(msg):
    data = str(msg).replace("data: ", "").replace('"', "").split(r"\r\n")
    return {
        "x": float(data[0].split("=")[1]),
        "y": float(data[1].split("=")[1]),
        "x_next": float(data[2].split("=")[1]),
        "y_next": float(data[3].split("=")[1]),
        "number": data[4].split("=")[1],
        "code": data[5].split("=")[1],
    }


def patrol(goal):
    global patrolling

    goal_pose = MoveBaseGoal()
    goal_pose.target_pose.header.frame_id = "map"
    goal_pose.target_pose.pose.position.x = goal[0]*0.81
    goal_pose.target_pose.pose.position.y = goal[1]*0.81
    goal_pose.target_pose.pose.position.z = 0
    goal_pose.target_pose.pose.orientation.x = 0
    goal_pose.target_pose.pose.orientation.y = 0
    goal_pose.target_pose.pose.orientation.z = 0
    goal_pose.target_pose.pose.orientation.w = 1

    print(goal_pose)
    client.send_goal(goal_pose)
    patrolling = True


def planning_status(data):
    global patrolling

    if data.status > 1:
        patrolling = False


def wander():
    global cmd_vel_pub

    twist = Twist()

    if g_range_ahead < 0.6:
        # TURN
        twist.linear.x = 0.0
        twist.angular.z = 0.4
    else:
        # FORWARD
        twist.linear.x = 0.3
        twist.angular.z = 0.0

    cmd_vel_pub.publish(twist)


def rotate():
    global cmd_vel_pub

    twist = Twist()
    twist.linear.x = 0.0
    twist.angular.z = 0.1

    cmd_vel_pub.publish(twist)


def stop():
    global cmd_vel_pub
    global planning_cancel_pub

    twist = Twist()
    twist.linear.x = 0.0
    twist.angular.z = 0.0

    cmd_vel_pub.publish(twist)
    planning_cancel_pub.publish()


def get_vector(distance):
    norm = math.sqrt(distance[0] ** 2 + distance[1] ** 2)
    direction = [distance[0] / norm, distance[1] / norm]
    return direction


def get_hidden_frame(qr1, qr2):
    global hidden_rot
    global hidden_trans

    a = np.array([[qr_database[qr1]["x"], qr_database[qr1]["y"], 0],
                  [qr_database[qr2]["x"], qr_database[qr2]["y"], 0]])

    b = np.array([[qr_database[qr1]["trans_map"][0], qr_database[qr1]["trans_map"][1], 0],
                  [qr_database[qr2]["trans_map"][0], qr_database[qr2]["trans_map"][1], 0]])

    v1 = np.array([a[0][0]-a[1][0], a[0][1]-a[1][1]])
    v2 = np.array([b[0][0]-b[1][0], b[0][1]-b[1][1]])

    vector_1 = get_vector(v1)
    vector_2 = get_vector(v2)
    unit_vector_1 = vector_1 / np.linalg.norm(vector_1)
    unit_vector_2 = vector_2 / np.linalg.norm(vector_2)
    dot_product = np.dot(unit_vector_1, unit_vector_2)
    angle = np.arccos(dot_product)

    hidden_rot = np.array([[math.cos(angle), -math.sin(angle), 0],
                           [math.sin(angle), math.cos(angle), 0],
                           [0, 0, 1]])

    centroid_a = np.mean(a, axis=0)
    centroid_b = np.mean(b, axis=0)
    hidden_trans = -np.matmul(hidden_rot, centroid_a) + centroid_b

    print("Hidden rotation angle is %s" % angle)
    print("Hidden translation is %s\n" % str(hidden_trans))


def transform_vector(goal_x, goal_y):
    global hidden_rot
    global hidden_trans

    goal_vector = np.array([goal_x, goal_y, 0])
    goal_vector = np.dot(goal_vector, hidden_rot.T) + \
        hidden_trans.reshape([1, 3])

    return goal_vector[0]


if __name__ == '__main__':
    # Init general data
    qr_detected = None
    qr_database = {}
    g_range_ahead = 1.0
    hidden_rot = []
    hidden_trans = []
    patrolling = False
    next_qr_number = None

    # Init node
    rospy.init_node('scanQR')
    qr_msg_sub = rospy.Subscriber(
        '/visp_auto_tracker/code_message', String, detect_qr)
    scan_sub = rospy.Subscriber('scan', LaserScan, scan_callback)
    planning_status_sub = rospy.Subscriber(
        'move_base/result', MoveBaseActionResult, planning_status)
    planning_cancel_pub = rospy.Publisher(
        'move_base/cancel', GoalID, queue_size=1)
    cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
    rate = rospy.Rate(10)

    # Tf utilities
    listener = tf.TransformListener()
    br = tf.TransformBroadcaster()

    # Planning utilities
    client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
    client.wait_for_server()

    while not rospy.is_shutdown():
        # Wander to find first two QR
        if len(qr_database) < 2:
            wander()

        # We can now get the hidden transformation
        elif len(qr_database) == 2 and len(hidden_trans) == 0:
            print("Finding hidden transformation..")
            keys = list(qr_database.keys())
            get_hidden_frame(keys[0], keys[1])

        # Use the hidden transformation to locate the missing QR codes
        elif len(qr_database) < 5 and not patrolling:
            print(qr_database)
            print("Hidden rotation angle is %s" % str(hidden_rot))
            print("Hidden translation is %s\n" % str(hidden_trans))
            # Prepare next QR to find
            if not next_qr_number:
                # Check found QR to pick next target
                for key in qr_database.keys():
                    qr_number = qr_database[key]["number"]
                    next_qr_number = int(qr_number) + 1
                    if next_qr_number == 6:
                        next_qr_number = 1
                    next_qr_number = str(next_qr_number)

                    if next_qr_number not in qr_database:
                        # Target is valid, start to patrol!
                        print("New target QR : %s" % (next_qr_number))

                        goal = transform_vector(
                            qr_database[qr_number]["x_next"],
                            qr_database[qr_number]["y_next"]
                        )
                        print("Starting patrolling -> " + str(goal) + "\n")
                        patrol(goal)
                        break
            # Goal position reached but QR not found, rotate to find it
            else:
                rotate()

        # Mission complete!
        elif len(qr_database) == 5:
            print("Hidden rotation angle is %s" % str(hidden_rot))
            print("Hidden translation is %s\n" % str(hidden_trans))
            # Decode msg!
            code = ""
            for i in range(1, 6):
                code += qr_database[str(i)]["code"]
            print("The secret message is: %s" % code)
            break

        # Detect QR
        if qr_detected and not qr_detected["number"] in qr_database:
            qr_msg = qr_detected
            trans_map = None
            rot_map = None

            qr_number = qr_msg["number"]
            print("QR DETECTED -> " + qr_number)

            # If searching for the first 2 QR, the map pos is needed for the hidden frame
            if len(qr_database) < 2:
                stop()
                rospy.sleep(2)

                qr_msg = rospy.wait_for_message(
                    '/visp_auto_tracker/code_message', String)
                # Once stopped check that we still see the QR. Otherwise the pos will be wrong
                if len(str(qr_msg)) < 40:
                    print("No msg detected. Continue\n")
                    continue
                # If another QR is seen, wait until next loop to check if already saved
                qr_msg = decode_msg(qr_msg)
                if qr_msg["number"] != qr_number:
                    print("Different qr detected. Continue\n")
                    continue

                qr_pos = rospy.wait_for_message(
                    '/visp_auto_tracker/object_position', PoseStamped)
                pos = qr_pos.pose.position
                ori = qr_pos.pose.orientation

                # Create transformations
                now = rospy.Time.now()
                br.sendTransform(
                    (pos.x, pos.y, pos.z),
                    (ori.x, ori.y, ori.z, ori.w),
                    now,
                    "QR_"+qr_number,
                    "camera_optical_link"
                )
                listener.waitForTransform(
                    "QR_"+qr_number,
                    "map",
                    now,
                    rospy.Duration(secs=1)
                )
                (trans_map, rot_map) = listener.lookupTransform(
                    "map",
                    "QR_"+qr_number,
                    now
                )

            # Save data
            qr_database[qr_msg["number"]] = qr_msg
            qr_database[qr_msg["number"]]["trans_map"] = trans_map
            qr_database[qr_msg["number"]]["rot_map"] = rot_map
            print("Saved!\n")

            # If patrolling (>2 qr)
            if next_qr_number and next_qr_number == qr_msg["number"]:
                stop()
                next_qr_number = None

        rate.sleep()
