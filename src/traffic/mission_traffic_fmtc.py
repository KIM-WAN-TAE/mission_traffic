#!/usr/bin/env python3
# -*-coding:utf-8-*-

import rospy
import sys, os
import time
import numpy as np
from macaron_6.msg import Traffic, obj_info
from std_msgs.msg import Bool, String



#names: {0: 'green', 1: 'yellow', 2: 'red', 3: 'all_green', 4: 'left', 5: 'delivery_a1', 6: 'delivery_a2', 7: 'delivery_a3', 8: 'delivery_b1', 9: 'delivery_b2', 10: 'delivery_b3'}

traffic_list = ['green',
                "yellow",
                "red", 
                'all_green', 
                "left"]

# Parameter
MODE = 0  # 미션에 따라 모드 설정해주기(0:직진 / 1:좌회전)
BEFORE_STOP_LINE_SPEED = 70
AFTER_STOP_LINE_SPEED = 110 - 15
detect_max_gap = 15
detect_min_gap = 3

class mission_traffic:
    def __init__(self, Place=1):
        #fmtc
        rospy.init_node('mission_traffic_node') 
        if Place==1:
           self.Straight_Stop_Line =  []
           self.Left_Stop_Line = []

        #k_city
        if Place==2:
            self.Straight_Stop_Line = []
            self.Left_Stop_Line = []

        self.sub_sign = rospy.Subscriber('/traffic_obj', Traffic, self.obj_callback, queue_size=1)
        self.pub_sign = rospy.Publisher('/sign', String, queue_size=1)
        
        self.stop = 0
        self.turn_left = 0
        self.go = 0
        self.traffic_flag = 1
        self.time_rec = 0
        self.traffic_s = 30.3
        self.done = False
        self.Stop_State = None
        self.start_count = 0
        self.go_count = 0
        self.stop_count = 0
        self.turn_left_count = 0
        # self.traffic = [0, 0, 0, 0, 0]
        self.mode = 0
        self.end_time = None
        self.interval_time_straight = 120
        self.interval_time_left = 120
             

    def obj_callback(self, data):
        print('stop cnt :' ,self.stop_count)
        print('stop : , ',self.stop)
        print('go cnt  : ', self.go_count)
        print('go : ' ,self.go)
        print('turn left : ' ,self.turn_left)
        print()

        for sign in data.obj:
        #     if sign.ns in traffic_list:  # 표지판인 경우만 detect
        #         if sign.ns == "left":
        #             # 넓이와 class 이름 tuple 형태로 추가
        #             left_box.append([sign.xmin, sign.ymin, sign.xmax, sign.ymax, sign.ns])
                
        #         else:
        #             # 넓이와 class 이름 tuple 형태로 추가
            #rest_boxes.append([sign.xmin, sign.ymin, sign.xmax, sign.ymax, sign.ns])
            # print(sign.ns)
            self.traffic_update(sign.ns)

        # # 좌회전검출시 좌회전만
        # if len(left_box) >= 1:

        #     left_box.sort(key=lambda x: x[3]) 

        #     if len(left_box) >= 4:
        #         self.traffic_update(left_box[0][4])
        #         self.traffic_update(left_box[1][4])
        #     else:
        #         self.traffic_update(left_box[0][4])
        
        # # 검출 안되면 나머지 박스들끼리만
        # else:

        #     rest_boxes.sort(key=lambda x: x[3]) 

        #     if len(rest_boxes) >= 4:
        #         self.traffic_update(rest_boxes[0][4])
        #         self.traffic_update(rest_boxes[1][4])
        #     else:
        #         self.traffic_update(rest_boxes[0][4])
                
    def traffic_update(self, ns):
        if ns == "red":
            self.stop += 1
            self.go = 0
            self.turn_left = 0

        elif ns == "yellow":
            self.stop += 1
            self.go = 0
            self.turn_left = 0

        elif ns == "green":
            self.stop = 0
            self.go += 1
            self.turn_left = 0

        elif ns == "all_green":
            self.stop = 0
            self.go += 1
            self.turn_left = 0

        elif ns == "left":
            self.stop = 0
            self.go = 0
            self.turn_left += 1

    def reset(self):
        self.stop = 0
        self.turn_left = 0
        self.go = 0
        self.time_rec = 0
        self.traffic_s = 30.3
        self.done = False
        self.Stop_State = True
        self.start_count = 0
        self.go_count = 0
        self.stop_count = 0
        self.turn_left_count = 0
        # self.traffic = [0, 0, 0, 0, 0]
        self.end_time = None
        self.interval_time_straight = 120
        self.interval_time_left = 120
             
    def reset_count(self):
        self.stop = 0
        self.turn_left = 0
        self.go = 0
   
    def run(self, s, mode):
        self.mode = mode
        speed = 0.0
        # erp와 정지선 사이의 거리를 토대로 인지판단 구간 갱신
        if self.mode == 0:
            for i in self.Straight_Stop_Line:
                if i - 16 <= s <= i + 10:
                    self.traffic_s = i

        if self.mode == 1:
            for i in self.Left_Stop_Line:
                if i - 16 <= s <= i + 10:
                    self.traffic_s = i


        # 신호등 인식 시작 구간
        if self.traffic_flag == 0:
            speed = 90

            # 인지판단 구간 진입
            # 만약 정지선(traffic_s)을 지났을때도 멈추도록 and 값
            if (detect_min_gap < self.traffic_s - s <detect_max_gap):
                print('정지에 대비하기 위해 감속 합니다!')
                self.traffic_flag = 1

        elif self.traffic_flag == 1:
            speed = BEFORE_STOP_LINE_SPEED

            # 직진인 경우
            if self.mode == 0:
                # 초록불(green, all_green)
                if self.go > 3:
                    self.stop_count = 0
                    self.go_count += 1
                    self.turn_left_count = 0
                    self.reset_count()

                # 빨간불(red) or 주황불(yellow)
                elif self.stop > 3:
                    self.stop_count += 1
                    self.go_count = 0
                    self.turn_left_count = 0
                    self.reset_count()

                # 좌회전(left)
                elif self.stop > 3:
                    self.stop_count += 1
                    self.go_count = 0
                    self.turn_left_count = 0
                    self.reset_count()

                if self.go_count >= 3:
                    self.Stop_State = False
                    self.reset_count()

            # 좌회전 미션인 경우
            elif self.mode == 1:

                # 좌회전(left) 
                if self.turn_left > 10:
                    self.stop_count = 0
                    self.go_count = 0
                    self.turn_left_count += 1
                    self.reset_count()

                # 빨간불(red), 주황불(orange)
                elif self.stop > 10:
                    self.stop_count += 1
                    self.go_count = 0
                    self.turn_left_count = 0
                    self.reset_count()

                # 초록불(green, all_green)
                if self.go > 10:
                    self.stop_count += 1
                    self.go_count = 0
                    self.turn_left_count = 0
                    self.reset_count()

                if self.turn_left_count >= 15:
                    self.Stop_State= False
                    self.reset_count()
            
            over_stop_line = 2.0
            if self.traffic_s - s <= over_stop_line:  # 인지판단 구간을 넘을 시
                self.traffic_flag = 2

                if self.Stop_State is True:
                    print("정지 신호")

                else:
                    print("직진 신호 혹은 좌회전 신호")
       
        elif self.traffic_flag == 2:
            if self.end_time is None:
                self.end_time = time.time()

            if self.mode == 0:
                if self.Stop_State is True and time.time() < self.end_time + self.interval_time_straight :
                    speed = 0
                    if self.go > 1:
                        self.start_count += 1
                    else:
                        self.start_count = 0

                    if self.start_count >= 3:
                        self.traffic_flag = 3
                else:
                    speed = AFTER_STOP_LINE_SPEED
                    self.traffic_flag = 3                
                    
            elif self.mode == 1:
                if self.Stop_State is True and time.time() < self.end_time + self.interval_time_left :
                    speed = 0
                    if self.turn_left > 1:
                        self.start_count += 1
                    else:
                        self.start_count = 0

                    if self.start_count >= 3:
                        self.traffic_flag = 3
                                        
                else:
                    speed = 90
                    self.traffic_flag = 3

            self.time_rec = time.time()
            

            

        # 초록불 판단 후
        elif self.traffic_flag == 3:
            speed = 90
            t = time.time()
            while(time.time()-t < 0.3):
                print("done!")
            self.done = True
            self.reset()
            print('신호등 미션 끝!\n')
        
        self.Stop_State = str(self.Stop_State)
        string_msg = String()
        string_msg.data = self.Stop_State
        print(string_msg.data)
        self.pub_sign.publish(string_msg)


def main():
    traffic = mission_traffic()
    while not rospy.is_shutdown():
        traffic.run(1, 0)


if __name__ == "__main__":
    main()
