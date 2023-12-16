from MavQuad.mavquad import DroneAPM, COLORED_RESULT
import time
import numpy as np
import cv2 as cv
import socket
import sys

if __name__ == "__main__":

    apm = DroneAPM("udpin:localhost:14551")

    print(f'Waiting heartbeat...')

    while True:
        if apm.connected:
            print("Drone connected!")
            break

    result = apm.setModeGuided()
    print(f'Result of setModeGuided command: {COLORED_RESULT(result)}')
    apm.sendGpOrigin()

    time.sleep(1)

    result = apm.arm()
    print(f'Result of arm command: {COLORED_RESULT(result)}')

    time.sleep(1)

    result = apm.takeoff(1)
    print(f'Result of takeoff command: {COLORED_RESULT(result)}')

    time.sleep(8)

    result = apm.setPosBody(10, 10, 0)
    print(f'Result of setPosBody command: {COLORED_RESULT(result)}')

    time.sleep(20)
    try:
        # 创建UDP套接字
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        
        # 允许地址重用
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

        # 设置超时时间为5秒
        s.settimeout(5)
    except socket.error as msg:
        print('创建套接字失败。错误代码：' + str(msg[0]) + ' 消息：' + msg[1])
        sys.exit()

    try:
        # 绑定套接字到本地地址和端口
        s.bind(('', 3333))
    except socket.error as msg:
        print('绑定失败。错误：' + str(msg[0]) + ': ' + msg[1])
        sys.exit()

    print('服务器正在监听')

    cap = cv.VideoCapture('02.mp4')
    is_video_end = False

    while True:
        ret, frame = cap.read()
        
        if(is_video_end):
            is_video_end = False
            print("Video end... Starting Drone Command")

            img = cv.imread(cv.samples.findFile("first.png"))

            cv.imshow("frame", img)
            cv.waitKey(1)

            try:

                # 接收数据
                d = s.recvfrom(1024)
                data = d[0]
            
                # 如果没有数据，则跳出循环
                if not data: 
                    print("No data")
            
                # 打印接收到的数据（去除首尾空格）
                print(data.strip())

                # send commands to drone
                font = cv.FONT_HERSHEY_SIMPLEX
                result = False

                if data == b'button Press':
                    result = apm.land(20)
                    print(f'Result of land command: {COLORED_RESULT(result)}')
                    cv.putText(img,'Land',(500,450), font, 4,(255,255,255),2,cv.LINE_AA)
                    cv.imshow("frame", img)
                    cv.waitKey(1)
                elif data == b'button Up':
                    result = apm.setPosBody(0.5, 0, 0)
                    print(f'Result of Forward command: {COLORED_RESULT(result)}')
                    cv.putText(img,'Forward',(500,450), font, 4,(255,255,255),2,cv.LINE_AA)
                    cv.imshow("frame", img)
                    cv.waitKey(1)
                elif data == b'button Down':
                    result = apm.setPosBody(-0.5, 0, 0)
                    print(f'Result of Down command: {COLORED_RESULT(result)}')
                    cv.putText(img,'Down',(500,450), font, 4,(255,255,255),2,cv.LINE_AA)
                    cv.imshow("frame", img)
                    cv.waitKey(1)
                elif data == b'button left':
                    result = apm.setPosBody(0, -0.5, 0)
                    print(f'Result of left command: {COLORED_RESULT(result)}')
                    cv.putText(img,'Left',(500,450), font, 4,(255,255,255),2,cv.LINE_AA)
                    cv.imshow("frame", img)
                    cv.waitKey(1)
                elif data == b'button Right':
                    result = apm.setPosBody(0, 0.5, 0)
                    print(f'Result of Right command: {COLORED_RESULT(result)}')
                    cv.putText(img,'Right',(500,450), font, 4,(255,255,255),2,cv.LINE_AA)
                    cv.imshow("frame", img)
                    cv.waitKey(1)

            except socket.timeout:
                print("timeout")
                continue

            time.sleep(3)


        if ret:
            cv.imshow("frame", frame)
            
        if cv.waitKey(1) == ord('q'):
            break
        
        if cap.get(cv.CAP_PROP_POS_FRAMES) == cap.get(cv.CAP_PROP_FRAME_COUNT):
            is_video_end = True
            cap.set(cv.CAP_PROP_POS_FRAMES, 0)

    cap.release()
    cv.destroyAllWindows()
    # 关闭套接字
    s.close()

