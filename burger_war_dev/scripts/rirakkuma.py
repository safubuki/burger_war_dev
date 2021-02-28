#!/usr/bin/env python
# -*- coding: utf-8 -*-
# -----------------------------------------------------------------------
# ファイル名 ：rirakkuma.py
# 機能概要  ：2019ROBOCON（予選用）
#           ・move_baseを使用
#           ・スタート/ゴール地点は"Start_Goal.csv"ファイルで設定する
# 作成日時  ：2019/08/19
# -----------------------------------------------------------------------

# Import
#   common
import rospy
import math
#   move_base
from move_base_msgs.msg import MoveBaseActionResult
from geometry_msgs.msg import PoseStamped
#   Twist
from geometry_msgs.msg import Twist
#   file
import csv  # csv file
import os   # file path
#   euler to quaternio
import tf
from geometry_msgs.msg import Quaternion

# Add ImageProcessing --- START ---
# use LaserScan
from sensor_msgs.msg import LaserScan

# use Camera
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np

# Image Process function
import imgProc        #function
from imgProc import * #class

# Add ImageProcessing --- END ---

import math
from tf import TransformListener
from geometry_msgs.msg import PointStamped
from visualization_msgs.msg import Marker, MarkerArray

import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

#camera_fov = 50.0
#camera_width = 640.0


# PythonでEnum的なことを実現
class MainState():
    STOP         = 0    # 停止
    EXEC_ACTION  = 1    # アクション実行
    MOVING       = 2    # 移動
    HUNTING      = 3    # 追跡

class RirakkumaBot():
    # クラス変数
    HUNT_CNT_NUM = 0    # HUNTING(追跡)状態に遷移する待ち回数 (2020.08.18 Wait無効)
    
    def __init__(self, bot_name="NoName"):
        ### Parameter Settings
        # bot name 
        self.name = bot_name

        # State
        self.main_state      = MainState.STOP   # メイン状態
        self.prev_main_state = MainState.STOP   # 前回メイン状態
        self.next_state      = MainState.STOP   # 次状態
        # CSV ファイルから取り出したデータ保存用リスト
        self.c_data          = []               # csvデータ
        self.c_data_cnt      = 0                # csvデータ順次取得のためのカウンタ
        # simple/goal用のシーケンス番号 ※これ無いとエラーになるため必要
        self.goal_seq_no     = 0
        # HUNTING(追跡)移行カウンタ
        self.hunting_cnt     = 0

        # Flags
        # 初期化フラグ
        self.initialize_flg   = False
        # ゴール到着フラグ
        self.goal_arrival_flg = False

        ### Publisher を ROS Masterに登録
        # Velocity
        self.vel_pub         = rospy.Publisher('cmd_vel', Twist,queue_size=1)
        self.pub_goal        = rospy.Publisher('move_base_simple/goal', PoseStamped, queue_size=1, latch=True)
        ### Subscriber を ROS Masterに登録
        self.sub_goal_result = rospy.Subscriber("move_base/result", MoveBaseActionResult, self.result_callback, queue_size=1)
        
        # Add ImageProcessing --- START ---
        # lidar scan subscriber
        self.scan = LaserScan()
        self.lidar_sub = rospy.Subscriber('scan', LaserScan, self.lidarCallback)

        # camera subscribver
        # for convert image topic to opencv obj
        self.img = None
        self.camera_preview = True
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber('image_raw', Image, self.imageCallback)
        #self.image_sub = rospy.Subscriber('/red_bot/image_raw', Image, self.imageCallback)

        #cImgProc instance
        self.proc = cImgProc()
        # Add ImageProcessing --- END ---

        self.client = actionlib.SimpleActionClient('move_base',MoveBaseAction)

    def calcTwist_center(self, center, depth, S):
        """目標が中心になるようTwistの値を設定する"""
        #depth [m]
        if center != -1:
            val = int(center / 16) #centerを0-4の10段階に
            # --- 近距離 --------------------------------
            if 0.3 > depth:
            #if 100 < S:  
                x  =  -0.2
                th =  0.0
            # --- 中距離 --------------------------------
            elif 0.6 > depth:
                if   val == 4:
                    x  =  0.0
                    th = -0.2

                elif val == 3:
                    x  =  0.1
                    th = -0.1

                elif val == 2:
                    x = 0.0
                    th =  0.0

                elif val == 1:
                    x  =  0.1
                    th =  0.1

                else:
                    x  =  0.0
                    th =  0.2
            # --- 遠距離 ---------------------------------------                
            #elif 1.0 > depth:
            else :         
                if   val == 4:
                    x  =  0.0
                    th = -0.2

                elif val == 3:
                    x  =  0.1
                    th = -0.1

                elif val == 2:
                    x = 0.15
                    th =  0.0

                elif val == 1:
                    x  =  0.1
                    th =  0.1

                else:
                    x  =  0.0
                    th =  0.2
            # else:
            #     x=0.0
            #     th=0.0
        # --- no detect green
        else :
            x  = 0
            th = 0 

        # 更新
        print("blue detect x,th=", x, th)
        twist = Twist()
        twist.linear.x = x; twist.linear.y = 0; twist.linear.z = 0
        twist.angular.x = 0; twist.angular.y = 0; twist.angular.z = th
        return twist

    def csv_data(self):
        """CSVファイルから座標を取得する"""
        # csvファイルをOpen
        csv_pass = os.path.dirname(__file__) + "/position_list.csv"
        csv_file = open(csv_pass, "r")
        # データ読み込み
        pos_data = csv.reader(csv_file, delimiter=",", doublequote=True, lineterminator="\r\n", quotechar='"', skipinitialspace=True)
        # 最初の一行をヘッダーとして取得
        header = next(pos_data)
        # 各行のデータを抜き出し
        for row in pos_data:
            # データ保存用のリストにcsvファイルから取得したデータを保存する
            # appendでリストに別のリストとして要素を追加する
            self.c_data.append(row)

    def vel_ctrl(self, line_x, line_y, ang_z):
        """publisher：cmd_vel Topic用（旋回で使用）"""
        vel_msg = Twist()
        vel_msg.linear.x = line_x
        vel_msg.linear.y = line_y
        vel_msg.angular.z = ang_z
        self.vel_pub.publish(vel_msg)

    def simple_goal_publish(self,pos_list):
        """publisher：move_base_simple/goal Topic用（引数はリスト型で渡す）"""
        # Goal Setting
        goal = PoseStamped()
        goal.header.seq = self.goal_seq_no
        goal.header.frame_id = "map"              # mapで座標系で指定する
        goal.header.stamp = rospy.Time.now()       # タイムスタンプは今の時間

        self.goal_seq_no += 1                      # シーケンス番号を更新

        # ** 位置座標
        goal.pose.position.x = float(pos_list[1])
        goal.pose.position.y = float(pos_list[2])
        goal.pose.position.z = 0
        # ** 回転方向
        # 度数をラジアンに変換
        degree_val = float(pos_list[3])
        radian_val = math.radians(degree_val)
        # オイラー角をクォータニオンに変換
        # RESPECT @hotic06 オイラー角をクォータニオンに変換・設定する
        quate = tf.transformations.quaternion_from_euler(0.0, 0.0, radian_val)
        goal.pose.orientation.x = quate[0]
        goal.pose.orientation.y = quate[1]
        goal.pose.orientation.z = quate[2]
        goal.pose.orientation.w = quate[3]
        # debug
        print(goal)
        # 実際にTopicを配信する
        self.pub_goal.publish(goal)

    def result_callback(self,goal_result):
        """call back：move base result (ゴール座標到着検知)"""
        if goal_result.status.status == 3:  # ゴールに到着
            self.goal_arrival_flg = True

    def lidarCallback(self, data):
        """call back：lider"""
        self.scan = data

    def imageCallback(self, data):
        """call back：camera image """
        # comvert image topic to opencv object
        try:
            self.img = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            print(e)

        # image processing
        # liderCallbackより先にimageCallbackがコールされIndexError例外に対応
        try:
            self.proc.imageProcess1(self.img, self.scan)
            #print('cwd=', self.proc.cwd)
        except IndexError as e:
            print(e)
            return

        # Show camera window
        if self.proc.debug_view == 1:           
            cv2.imshow("Camera", self.proc.img_div2)            
            cv2.waitKey(1)

        # Show debug window
        if self.proc.debug_view == 2:
            #cv2.imshow("rila", self.proc.rila_img)
            #cv2.imshow("div2", self.proc.img_div2)            
            #cv2.imshow("div8", self.proc.img_div8)
            #cv2.imshow("red", self.proc.red_img)
            #cv2.imshow("green", self.proc.green_img)
            #cv2.imshow("blue", self.proc.blue_img)                        
            #cv2.imshow("Camera", self.proc.img)            
            cv2.imshow("debug1", self.proc.debug1_img)                        
            # --- add T.Ishigami 2020.03.15 22:40 ---
            # Add vertical window position for QVGA ( 960 - 260 = 700)
            # cv2.moveWindow("debug1", 0, 700)
            # Add vertical window position for RHC  ( 900 - 260 = 640)
            cv2.moveWindow("debug1", 0, 640)

            cv2.waitKey(1)
        # green_index = self.proc.green_center
        # if green_index != -1:
        #     green_distance = self.proc.depth_img.item(0,green_index,0)
        # else:
        #     green_distance = 0


    def is_start_hunting(self):
        """HUNTING(追跡)を開始するか判定する

        ・敵を発見したら「HUNT_CNT_NUM」回数待つ
        ・待ち回数を満たしたら現状態を保持して、次状態をHUNTING(追跡)にする
        
        戻り値  True：開始する/False：開始しない
        """
        if self.proc.green_center != -1:
            if self.hunting_cnt >= RirakkumaBot.HUNT_CNT_NUM:
                self.prev_main_state = self.main_state
                self.next_state      = MainState.HUNTING
                return True
            else:
                self.hunting_cnt += 1
        else:
            self.hunting_cnt = 0

        return False

    def is_finish_hunting(self):
        """HUNTING(追跡)を終了するか判定する

        ・敵を喪失したらHUNTING(追跡)状態に遷移する前状態に戻る
        
        戻り値  True：終了する/False：終了しない
        """
        if self.proc.green_center == -1:
            self.next_state = self.prev_main_state
            return True

        return False

    def func_state_stop(self):
        """状態処理関数：STOP(停止)"""
        # 初期処理未実施なら、次状態はEXEC_ACTION
        if self.initialize_flg == False:
            self.initialize_flg = True
            self.next_state     = MainState.EXEC_ACTION

    def func_state_exec_action(self):
        """状態処理関数：EXEC_ACTION(アクション実行)"""
        # HUNTING(追跡)を開始するか判定する
        if self.is_start_hunting():
            # 開始する場合、以降の処理はしない
            return

        # アクションリストを読み込み
        pos_info         = self.c_data[self.c_data_cnt]
        self.c_data_cnt += 1 
        # アクションリストに基づいてアクション
        if pos_info[0]   == "move":
            # 目的地に移動 (次状態はMOVING)
            self.simple_goal_publish(pos_info)
            self.next_state = MainState.MOVING
        elif pos_info[0] == "turn": 
            # 旋回         (状態維持)
            # 度数をラジアンに変換
            degree_val = float(pos_info[3])
            radian_val = math.radians(degree_val)
            self.vel_ctrl(0,0,radian_val)
        else:
            # 意図しないアクションの場合は次のリスト
            pass

    def func_state_moving(self):
        """状態処理関数：MOVING(移動)"""
        # HUNTING(追跡)を開始するか判定する
        if self.is_start_hunting():
            # 開始する場合、以降の処理はしない
            return

        # 目的地に到着したら、次状態はEXEC_ACTION
        if self.goal_arrival_flg == True:
            self.goal_arrival_flg = False
            self.next_state       = MainState.EXEC_ACTION

    def func_state_hunting(self):
        """状態処理関数：HUNTING(追跡)"""
        # HUNTING(追跡)を終了するか判定する
        if self.is_finish_hunting():
            # 終了する場合、以降の処理は実施しない
            return

        # 敵の追跡を実行
        print("detect green")
        self.client.cancel_goal()
        twist = self.calcTwist_center(self.proc.green_center, self.proc.green_center_depth, self.proc.green_center_S)
        print("#################### green_S_depth ####################")
        print(self.proc.green_center_S, "-", self.proc.green_center_depth)
        print("#######################################################")                
        self.vel_pub.publish(twist)
        print("snipe_enemy")

    def strategy(self):
        """ロボット動作メイン処理（ステートマシンで制御）"""
        while not rospy.is_shutdown():
            # メイン状態処理を行う
            if self.main_state == MainState.STOP:
                # 停止
                self.func_state_stop()
            elif self.main_state == MainState.EXEC_ACTION:
                # アクション実行
                self.func_state_exec_action()
            elif self.main_state == MainState.MOVING:
                # 移動
                self.func_state_moving()
            elif self.main_state == MainState.HUNTING:
                # 追跡
                self.func_state_hunting()
            else:
                pass

            # DEBUG Print
            print('main_state = ',self.main_state)
            print('next_state = ',self.next_state)

            # メイン状態を次の状態に更新
            self.main_state = self.next_state
            # 1秒Wait
            rospy.sleep(1)

if __name__ == "__main__":
    rospy.init_node('rirakkuma_node')
    bot = RirakkumaBot('rirakkuma')
    bot.csv_data()
    rospy.sleep(1.0)  # 起動後、ウェイト（調整値）
    bot.strategy()