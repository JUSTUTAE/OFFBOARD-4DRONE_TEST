#!/usr/bin/env python3

import rospy
import math
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest

# 형성된 형태의 간격 및 기타 매개변수 설정
formation_spacing = rospy.get_param("formation_spacing", 3.0)  # 형성된 형태의 드론 간격을 설정합니다.
max_speed = 0.8  # 드론의 최대 속도를 설정합니다.
deceleration_percentage = 0.9  # 목표 지점 90%까지 도달하면 감속 시작합니다.
current_state = State()  # 현재 드론 상태를 저장할 변수를 초기화합니다.
leader_pose = PoseStamped()  # 리더 드론의 자세를 저장할 변수를 초기화합니다.
leader_target_x = 8.0  # 리더 드론의 목표 X 위치를 설정합니다.
leader_target_y = 3.0  # 리더 드론의 목표 Y 위치를 설정합니다.
leader_target_z = 5.0  # 리더 드론의 목표 Z 위치를 설정합니다.
leader_current_x = 0.0  # 리더 드론의 현재 X 위치를 초기화합니다.
leader_current_y = 0.0  # 리더 드론의 현재 Y 위치를 초기화합니다.
leader_current_z = 0.0  # 리더 드론의 현재 Z 위치를 초기화합니다.

# 상태 콜백 함수
def state_cb(msg):
    global current_state
    current_state = msg

# 리더의 자세 콜백 함수
def leader_pose_cb(msg):
    global leader_pose, leader_current_x, leader_current_y
    leader_pose = msg
    leader_current_x = msg.pose.position.x
    leader_current_y = msg.pose.position.y

# 형태 업데이트 함수
def update_formation(current_pose, leader_pose, drone_id):
    num_columns = 2
    row = drone_id // num_columns  # 드론의 행 위치 계산
    col = drone_id % num_columns   # 드론의 열 위치 계산
    delta_x = col * formation_spacing  # X 방향으로의 간격 설정
    delta_y = row * formation_spacing  # Y 방향으로의 간격 설정
    current_pose.pose.position.x = leader_pose.pose.position.x + delta_x  # 드론의 X 위치 업데이트
    current_pose.pose.position.y = leader_pose.pose.position.y + delta_y  # 드론의 Y 위치 업데이트

if __name__ == "__main__":
    rospy.init_node("formation_flight")  # ROS 노드를 초기화합니다.

    num_drones = 4  # 사용할 드론 수를 설정합니다.

    state_subs = []  # 상태 메시지를 구독하는 객체를 저장할 리스트를 초기화합니다.
    local_pos_pubs = []  # 로컬 좌표 메시지를 게시하는 객체를 저장할 리스트를 초기화합니다.
    arming_clients = []  # Arming 명령을 보내는 클라이언트 객체를 저장할 리스트를 초기화합니다.
    set_mode_clients = []  # 모드 변경 명령을 보내는 클라이언트 객체를 저장할 리스트를 초기화합니다.

    for i in range(num_drones):
        state_subs.append(rospy.Subscriber("uav{}/mavros/state".format(i), State, callback=state_cb))  # 상태 메시지 구독자 생성
        local_pos_pubs.append(rospy.Publisher("uav{}/mavros/setpoint_position/local".format(i), PoseStamped, queue_size=10))  # 로컬 좌표 메시지 게시자 생성
        arming_clients.append(rospy.ServiceProxy("uav{}/mavros/cmd/arming".format(i), CommandBool))  # Arming 클라이언트 생성
        set_mode_clients.append(rospy.ServiceProxy("uav{}/mavros/set_mode".format(i), SetMode))  # 모드 변경 클라이언트 생성

    leader_pose_sub = rospy.Subscriber("uav0/mavros/local_position/pose", PoseStamped, callback=leader_pose_cb)  # 리더 드론의 자세 정보 구독

    rate = rospy.Rate(20)  # 주기적인 작업을 위한 Rate 객체 생성

    while not rospy.is_shutdown() and not current_state.connected:  # ROS가 종료되지 않고 드론 연결이 되지 않을 동안 대기
        rate.sleep()

    offb_set_mode = SetModeRequest()
    offb_set_mode.custom_mode = 'OFFBOARD'  # OFFBOARD 모드로 변경하기 위한 요청 객체 생성

    arm_cmd = CommandBoolRequest()
    arm_cmd.value = True  # Arming을 활성화하기 위한 요청 객체 생성

    last_req = rospy.Time.now()

    start_time = rospy.Time.now()
    current_time = start_time

    reached_target = False  # 목표 지점에 도달했는지 여부를 나타내는 플래그

    pose = PoseStamped()  # 로컬 좌표 메시지를 저장할 변수 초기화
    pose.pose.position.x = 0.0
    pose.pose.position.y = 0.0
    pose.pose.position.z = 0.0

    while not rospy.is_shutdown() and not reached_target:
        if current_state.mode != "OFFBOARD" and (rospy.Time.now() - last_req) > rospy.Duration(5.0):
            for i in range(num_drones):
                if set_mode_clients[i].call(offb_set_mode).mode_sent:
                    rospy.loginfo("OFFBOARD 모드 활성화")  # OFFBOARD 모드로 변경하는 로그 메시지 출력
            last_req = rospy.Time.now()
        else:
            if not current_state.armed and (rospy.Time.now() - last_req) > rospy.Duration(5.0):
                for i in range(num_drones):
                    if arming_clients[i].call(arm_cmd).success:
                        rospy.loginfo("uav {} 비행 준비 완료.".format(i))  # 드론 Arming 완료 로그 메시지 출력
                last_req = rospy.Time.now()

        current_time = rospy.Time.now()
        elapsed_time = current_time - start_time
        displacement = max_speed * elapsed_time.to_sec()  # 경과 시간에 따른 이동 거리 계산

        distance_to_target = math.sqrt((leader_target_x - leader_current_x)**2 + (leader_target_y - leader_current_y)**2)  # 리더와 목표 지점 간의 거리 계산

        # 90%까지 도달하면 감속 시작
        if distance_to_target < deceleration_percentage * displacement:
            max_speed = deceleration_percentage * max_speed  # 목표 지점에 가까워질수록 속도를 감소시킵니다.

        leader_pose.pose.position.x = leader_target_x
        leader_pose.pose.position.y = leader_target_y  # 리더 드론의 목표 위치 업데이트

        for i in range(num_drones):
            update_formation(pose, leader_pose, i)  # 형태를 업데이트하여 각 드론의 위치 설정

        if displacement >= distance_to_target:
            reached_target = True
            for i in range(num_drones):
                pose.pose.position.x = leader_target_x
                pose.pose.position.y = leader_target_y
                pose.pose.position.z = leader_target_z  # 목표 지점에 도달하면 드론의 위치를 목표 지점으로 설정하고 시작 시간을 다시 설정합니다.
            start_time = rospy.Time.now()

        for i in range(num_drones):
            local_pos_pubs[i].publish(pose)  # 드론의 로컬 좌표 메시지를 게시하여 움직이도록 합니다.
        rate.sleep()

    while not rospy.is_shutdown():
        for i in range(num_drones):
            local_pos_pubs[i].publish(pose)  # 목표 지점에 도달한 후에도 드론의 위치를 유지하도록 게시합니다.
        rate.sleep()
