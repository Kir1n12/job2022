import threading
import math
from dobot_api import DobotApiDashboard, DobotApi, DobotApiMove, MyType
from time import sleep
import numpy as np

# 全局变量(当前坐标)
current_actual = None

def connect_robot():
    try:
        ip = "ip_number"
        dashboard_p = 29999
        move_p = 30003
        feed_p = 30004
        print("正在建立连接...")
        dashboard = DobotApiDashboard(ip, dashboard_p)
        move = DobotApiMove(ip, move_p)
        feed = DobotApi(ip, feed_p)
        print(">.<连接成功>!<")
        return dashboard, move, feed
    except Exception as e:
        print(":(连接失败:(")
        raise e

def run_point(move: DobotApiMove, point_list: list):
    move.MovL(point_list[0], point_list[1], point_list[2], point_list[3])

def get_feed(feed: DobotApi):
    global current_actual
    hasRead = 0
    while True:
        data = bytes()
        while hasRead < 1440:
            temp = feed.socket_dobot.recv(1440 - hasRead)
            if len(temp) > 0:
                hasRead += len(temp)
                data += temp
        hasRead = 0

        a = np.frombuffer(data, dtype=MyType)
        if hex((a['test_value'][0])) == '0x123456789abcdef':

            # Refresh Properties
            current_actual = a["tool_vector_actual"][0]
            #print("tool_vector_actual:", current_actual)

        sleep(0.001)

def wait_arrive(point_list):
    global current_actual
    while True:
        is_arrive = True

        if current_actual is not None:
            for index in range(4):
                if (abs(current_actual[index] - point_list[index]) > 1.5):
                    is_arrive = False

            if is_arrive:
                return

        sleep(0.001)
    
def SpeedL(self, speed):
        """
        Set the cartesian acceleration ratio (Only for MovL, MovLIO, MovLR, Jump, Arc, Circle commands)
        speed : Cartesian acceleration ratio (Value range:1~100)
        """
        string = "SpeedL({:d})".format(speed)
        self.send_data(string)
        return self.wait_reply()

def zigzag(point,x,y,z,r,theta):
    k = 0
    l = 4
    d1 = 60 #[mm]
    d2 = 10 #[mm]
    theta_cos = math.cos(math.radians(theta))
    theta_sin = math.sin(math.radians(theta))

    for k in range(l):
        x = x + d1*theta_cos
        y = y + d1*theta_sin
        point = [x,y,z,r]
        print("1:",point)
        run_point(move, point)
        sleep(0.5)
        #wait_arrive(point_n)

        x = x - d2*theta_sin
        y = y + d2*theta_cos
        point = [x,y,z,r]
        print("2:",point)
        run_point(move, point)
        sleep(0.5)
        #wait_arrive(point_n)

        x = x - d1*theta_cos
        y = y - d1*theta_sin
        point = [x,y,z,r]
        print("3:",point)
        run_point(move, point)
        sleep(0.5)
        #wait_arrive(point_n)

        x = x - d2*theta_sin
        y = y + d2*theta_cos
        point = [x,y,z,r]
        print("4:",point)
        run_point(move, point)
        sleep(0.5)
        #wait_arrive(point_n)

if __name__ == '__main__':
    dashboard, move, feed = connect_robot()
    dashboard.EnableRobot()
    feed_thread = threading.Thread(target=get_feed, args=(feed,))
    feed_thread.setDaemon(True)
    feed_thread.start()
    
    r = 200
    init_x = 200
    init_y = 0
    init_z = 0
    init_r = 0

    point_n = [init_x,0,0,0]
    run_point(move, point_n)
    sleep(0.5) #[sec]
    #wait_arrive(point_n)

    theta = 0
    zigzag(point_n,init_x,init_y,init_z,init_r,theta)

    theta = 30
    init_x = r*math.cos(math.radians(theta))
    init_y = r*math.sin(math.radians(theta))
    point_n2 = [init_x,init_y,0,0]

    run_point(move,point_n2)
    zigzag(point_n2,init_x,init_y,init_z,init_r,theta)

# using the code for  © 2024 Salmontech Inc.　 https://www.salmontech.jp/ 
