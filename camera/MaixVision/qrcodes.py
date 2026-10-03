from maix import display, app, time, image, uart,camera
import cv2
import sys


def qr_task():
    question = None

    cap = cv2.VideoCapture(0)
    cap.set(cv2.CAP_PROP_FPS, 30)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)


    if not cap.isOpened():
        print("无法打开摄像头")
        return

    count_qrcodes=0              #记数标识
    last_qiestion=None

    print("开始读取")
    while not app.need_exit():
        question = None
        ret, frame = cap.read()

        if not ret:
            print("无法读取帧")
            break

        img_2 = image.cv2image(frame, bgr=True, copy=False)

        if(count_qrcodes<3):                #判断是否连续三次
            qrcodes = img_2.find_qrcodes()
            for qr in qrcodes:
                corners = qr.corners()

                for i in range(4):
                    img_2.draw_line(corners[i][0], corners[i][1],
                                    corners[(i + 1) % 4][0], corners[(i + 1) % 4][1],
                                    image.COLOR_RED)

                question = qr.payload()
                img_2.draw_string(qr.x(), qr.y() - 15, question, image.COLOR_RED)
            if(count_qrcodes==0 and question!=None):
                last_qiestion=question
                count_qrcodes += 1
            elif(count_qrcodes!=0 and last_qiestion==question and question!=None):
                count_qrcodes += 1

        disp.show(img_2)

        fps = time.fps()                                          #输出帧率大小
        print(f"time:{1000/fps:.2f}ms, fps:{fps:.2f}")

        if last_qiestion is not None and count_qrcodes==3:
            serial_dev.write_str(f"The number is {last_qiestion} MaixPy\n")         

        #data = serial_dev.read()

        if last_qiestion is not None and count_qrcodes==3:
            data=b"ok\n"
        else:
            data=None

        if data:
            text = data.decode().strip()
            print("receive:", text)

            if text == "ok":
                print("QR task finished")
                cap.release()
                return
    cap.release()


#------------------------------------------------------------------------------主函数
cam = camera.Camera(512,320)             
disp = display.Display()                              
serial_dev = uart.UART("/dev/ttyS4", 115200)
qr_task()

while not app.need_exit():
    img = cam.read()
    disp.show(img)






    #opencv部分
    #img_cv=image.image2cv(img,False,False)
    #gray=cv2.cvtColor(img_cv,cv2.COLOR_RGB2GRAY)
    #showa=image.cv2image(gray,False,False)







