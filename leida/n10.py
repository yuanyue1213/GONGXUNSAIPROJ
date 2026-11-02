from __future__ import print_function
import serial,math,heapq,cv2
import numpy as np
# ============================================================
# 1. N10 串口参数
# ============================================================
PORT = "COM13"
BAUDRATE = 230400
SERIAL_TIMEOUT = 5
# ============================================================
# 2. 场地参数
# ============================================================
MAP_WIDTH_MM = 2400
MAP_HEIGHT_MM = 2400
GRID_SIZE_MM = 50
GRID_W = MAP_WIDTH_MM // GRID_SIZE_MM
GRID_H = MAP_HEIGHT_MM // GRID_SIZE_MM
# ============================================================
# 3. 机器人参数
# ============================================================
ROBOT_SIZE_MM = 300
ROBOT_GRID_SIZE = ROBOT_SIZE_MM // GRID_SIZE_MM
ROBOT_HALF_GRID = ROBOT_GRID_SIZE // 2
# ============================================================
# 4. A*参数
# ============================================================
MOVE_COST = 1.0
TURN_COST = 12.0
DIRECTIONS = [(1,0),(-1,0),(0,1),(0,-1)]
# ============================================================
# 5. 启停区
# ============================================================
START_ZONE_BOTTOM_RIGHT = 1
START_ZONE_TOP_RIGHT = 2
CURRENT_START_ZONE = START_ZONE_BOTTOM_RIGHT
BOTTOM_RIGHT_ZONE = (2100,0,2400,300)
TOP_RIGHT_ZONE = (2100,2100,2400,2400)
BOTTOM_RIGHT_LIDAR_POS = (2250.0,150.0)
TOP_RIGHT_LIDAR_POS = (2250.0,2250.0)
# ============================================================
# 6. A*目标点
# ============================================================
TARGET_NODE = (24,24)
# ============================================================
# 7. 固定禁止区域
# ============================================================
PROCESS_BLOCKS = [(550,550,1000,1000),(1400,550,1850,1000),(550,1400,1000,1850),(1400,1400,1850,1850)]
TEMP_ZONE = (0,910,150,1490)
ROUGH_ZONE = (910,0,1490,150)
# ============================================================
# 8. 地图
# ============================================================
known_grid = np.zeros((GRID_H,GRID_W),dtype=np.uint8)
lidar_grid = np.zeros((GRID_H,GRID_W),dtype=np.uint8)
planning_grid = np.zeros((GRID_H,GRID_W),dtype=np.uint8)
# ============================================================
# 9. 当前雷达点云
# ============================================================
CURRENT_SCAN_POINTS = []
LAST_START_ANGLE = None
FRAME_COUNT = 0
# ============================================================
# 10. OpenCV
# ============================================================
CELL_PIXELS = 18
WINDOW_W = GRID_W * CELL_PIXELS
WINDOW_H = GRID_H * CELL_PIXELS
# ============================================================
# 11. 固定地图
# ============================================================
def mark_rect(grid,x0,y0,x1,y1,value):
    gx0 = int(math.floor(x0/GRID_SIZE_MM))
    gy0 = int(math.floor(y0/GRID_SIZE_MM))
    gx1 = int(math.ceil(x1/GRID_SIZE_MM))
    gy1 = int(math.ceil(y1/GRID_SIZE_MM))
    gx0,gy0 = max(0,gx0),max(0,gy0)
    gx1,gy1 = min(GRID_W,gx1),min(GRID_H,gy1)
    grid[gy0:gy1,gx0:gx1]=value

def get_start_node():
    return (45,3) if CURRENT_START_ZONE==START_ZONE_BOTTOM_RIGHT else (45,45)

def build_known_map():
    known_grid[:,:]=0
    for rect in PROCESS_BLOCKS:
        mark_rect(known_grid,*rect,1)
    mark_rect(known_grid,*TEMP_ZONE,1)
    mark_rect(known_grid,*ROUGH_ZONE,1)
    mark_rect(known_grid,*BOTTOM_RIGHT_ZONE,1)
    mark_rect(known_grid,*TOP_RIGHT_ZONE,1)
    if CURRENT_START_ZONE==START_ZONE_BOTTOM_RIGHT:
        mark_rect(known_grid,*BOTTOM_RIGHT_ZONE,0)
    else:
        mark_rect(known_grid,*TOP_RIGHT_ZONE,0)
# ============================================================
# 12. N10官方示例的数据解析方式
# ============================================================
def parse_data(data):
    speed = data[0]*256+data[1]
    start_angle = (data[2]*256+data[3])/100.0
    distances_and_intensities=[]
    for x in range(4,50,3):
        distance = data[x]*256+data[x+1]
        intensity = data[x+2]
        distances_and_intensities.append((distance,intensity))
    end_angle = (data[52]*256+data[53])/100.0
    return speed,start_angle,distances_and_intensities,end_angle
# ============================================================
# 13. N10一帧16点 -> 雷达xy
# ============================================================
def is_target_angle(angle):
    angle%=360.0
    return angle>=270.0 or angle<=90.0

def calculate_lidar_xy(start_angle,end_angle,distances_and_intensities):
    points=[]
    if end_angle<start_angle:
        end_angle+=360.0
    angle_step=(end_angle-start_angle)/15.0
    for i,(distance,intensity) in enumerate(distances_and_intensities):
        if distance<=0:
            continue
        angle=(start_angle+i*angle_step)%360.0
        if not is_target_angle(angle):
            continue
        rad=math.radians(angle)
        x_lidar=distance*math.cos(rad)
        y_lidar=distance*math.sin(rad)
        points.append((x_lidar,y_lidar))
    return points
# ============================================================
# 14. 雷达 -> 地图 TF
# ============================================================
def lidar_to_map(x_lidar,y_lidar):
    if CURRENT_START_ZONE==START_ZONE_BOTTOM_RIGHT:
        x0,y0 = BOTTOM_RIGHT_LIDAR_POS
        x_map = x0 - y_lidar
        y_map = y0 + x_lidar
    else:
        x0,y0 = TOP_RIGHT_LIDAR_POS
        x_map = x0 + y_lidar
        y_map = y0 - x_lidar
    return x_map,y_map
# ============================================================
# 15. 地图毫米坐标 -> 栅格
# ============================================================
def map_xy_to_grid(x_map,y_map):
    if not (0<=x_map<MAP_WIDTH_MM and 0<=y_map<MAP_HEIGHT_MM):
        return None
    gx=int(x_map//GRID_SIZE_MM)
    gy=int(y_map//GRID_SIZE_MM)
    return gx,gy
# ============================================================
# 16. 按官方示例方式读取一帧
# ============================================================
def read_n10_frame(ser):
    while True:
        data=ser.read(1)
        if len(data)==0:
            return None
        if data[0]==0xA5:
            data=ser.read(1)
            if len(data)==0:return None
            if data[0]==0x5A:
                data=ser.read(1)
                if len(data)==0:return None
                if data[0]==0x3A:
                    data=ser.read(55)
                    if len(data)!=55:
                        return None
                    return parse_data(data)
# ============================================================
# 17. 把真实N10帧加入当前点云
# ============================================================
def process_n10_frame(speed,start_angle,distances,end_angle):
    global CURRENT_SCAN_POINTS,LAST_START_ANGLE,FRAME_COUNT
    FRAME_COUNT+=1
    if LAST_START_ANGLE is not None:
        if (LAST_START_ANGLE-start_angle)>100:
            CURRENT_SCAN_POINTS=[]
    LAST_START_ANGLE=start_angle
    lidar_points=calculate_lidar_xy(start_angle,end_angle,distances)
    for x_lidar,y_lidar in lidar_points:
        x_map,y_map=lidar_to_map(x_lidar,y_lidar)
        if 0<=x_map<MAP_WIDTH_MM and 0<=y_map<MAP_HEIGHT_MM:
            CURRENT_SCAN_POINTS.append((x_map,y_map))
# ============================================================
# 18. 点云 -> 50mm栅格
# ============================================================
def rebuild_lidar_grid():
    lidar_grid[:,:]=0
    for x_map,y_map in CURRENT_SCAN_POINTS:
        res=map_xy_to_grid(x_map,y_map)
        if res is None:continue
        gx,gy=res
        lidar_grid[gy,gx]=1

def rebuild_planning_grid():
    planning_grid[:,:]=np.maximum(known_grid,lidar_grid)
# ============================================================
# 19. 机器人6x6碰撞检测
# ============================================================
def robot_can_stand(cx,cy,grid):
    min_x=cx-ROBOT_HALF_GRID
    max_x=cx+ROBOT_HALF_GRID-1
    min_y=cy-ROBOT_HALF_GRID
    max_y=cy+ROBOT_HALF_GRID-1
    if min_x<0 or min_y<0 or max_x>=GRID_W or max_y>=GRID_H:
        return False
    for gy in range(min_y,max_y+1):
        for gx in range(min_x,max_x+1):
            if grid[gy,gx]!=0:
                return False
    return True
# ============================================================
# 20. A*
# ============================================================
def heuristic(a,b):
    return abs(a[0]-b[0])+abs(a[1]-b[1])

def reconstruct_path(came_from,state):
    path=[(state[0],state[1])]
    while state in came_from:
        state=came_from[state]
        path.append((state[0],state[1]))
    path.reverse()
    return path

def astar(start,goal,grid):
    if not robot_can_stand(start[0],start[1],grid):return None
    if not robot_can_stand(goal[0],goal[1],grid):return None
    start_state=(start[0],start[1],-1)
    open_heap=[]
    heapq.heappush(open_heap,(heuristic(start,goal),0,start[0],start[1],-1))
    g_score={start_state:0}
    came_from={}
    closed=set()
    while open_heap:
        f,g,x,y,direction=heapq.heappop(open_heap)
        state=(x,y,direction)
        if state in closed:continue
        closed.add(state)
        if (x,y)==goal:return reconstruct_path(came_from,state)
        for new_direction,(dx,dy) in enumerate(DIRECTIONS):
            nx,ny=x+dx,y+dy
            if not robot_can_stand(nx,ny,grid):continue
            move_cost=MOVE_COST
            if direction!=-1 and direction!=new_direction:
                move_cost+=TURN_COST
            new_g=g+move_cost
            next_state=(nx,ny,new_direction)
            if next_state not in g_score or new_g<g_score[next_state]:
                g_score[next_state]=new_g
                came_from[next_state]=state
                h=heuristic((nx,ny),goal)
                heapq.heappush(open_heap,(new_g+h,new_g,nx,ny,new_direction))
    return None
# ============================================================
# 21. 路径简化
# ============================================================
def merge_same_direction(path):
    if path is None:return None
    if len(path)<=2:return path[:]
    result=[path[0]]
    last_dx=path[1][0]-path[0][0]
    last_dy=path[1][1]-path[0][1]
    for i in range(1,len(path)-1):
        dx=path[i+1][0]-path[i][0]
        dy=path[i+1][1]-path[i][1]
        if dx!=last_dx or dy!=last_dy:
            result.append(path[i])
        last_dx,last_dy=dx,dy
    result.append(path[-1])
    return result

def straight_line_is_safe(start,end,grid):
    x1,y1=start
    x2,y2=end
    if y1==y2:
        if x1==x2:
            return robot_can_stand(x1,y1,grid)
        step=1 if x2>x1 else -1
        for x in range(x1,x2+step,step):
            if not robot_can_stand(x,y1,grid):return False
        return True
    if x1==x2:
        step=1 if y2>y1 else -1
        for y in range(y1,y2+step,step):
            if not robot_can_stand(x1,y,grid):return False
        return True
    return False

def straighten_path(path,grid):
    if path is None:return None
    if len(path)<=2:return path[:]
    result=[path[0]]
    current_index=0
    while current_index<len(path)-1:
        farthest=current_index+1
        for j in range(len(path)-1,current_index,-1):
            s,e=path[current_index],path[j]
            if s[0]!=e[0] and s[1]!=e[1]:continue
            if straight_line_is_safe(s,e,grid):
                farthest=j
                break
        result.append(path[farthest])
        current_index=farthest
    return result

def validate_path(path,grid):
    if path is None:return False
    for x,y in path:
        if not robot_can_stand(x,y,grid):return False
    for i in range(len(path)-1):
        if not straight_line_is_safe(path[i],path[i+1],grid):return False
    return True

def plan_path(start,goal,grid):
    raw_path=astar(start,goal,grid)
    if raw_path is None:return None,None
    merged=merge_same_direction(raw_path)
    final_path=straighten_path(merged,grid)
    if not validate_path(final_path,grid):
        return raw_path,None
    return raw_path,final_path
# ============================================================
# 22. OpenCV坐标
# ============================================================
def node_to_pixel(x,y):
    return int(x*CELL_PIXELS),int(WINDOW_H-y*CELL_PIXELS)

def map_mm_to_pixel(x,y):
    return int(x/GRID_SIZE_MM*CELL_PIXELS),int(WINDOW_H-y/GRID_SIZE_MM*CELL_PIXELS)
# ============================================================
# 23. 显示
# ============================================================
def draw_map(raw_path,final_path,start,goal):
    img=np.ones((WINDOW_H+1,WINDOW_W+1,3),dtype=np.uint8)*255
    for gy in range(GRID_H):
        for gx in range(GRID_W):
            x1=gx*CELL_PIXELS
            y1=WINDOW_H-(gy+1)*CELL_PIXELS
            x2=(gx+1)*CELL_PIXELS
            y2=WINDOW_H-gy*CELL_PIXELS
            if known_grid[gy,gx]==1:
                cv2.rectangle(img,(x1,y1),(x2,y2),(70,70,70),-1)
            if lidar_grid[gy,gx]==1:
                cv2.rectangle(img,(x1,y1),(x2,y2),(0,180,255),-1)
    for x in range(GRID_W+1):
        px=x*CELL_PIXELS
        cv2.line(img,(px,0),(px,WINDOW_H),(220,220,220),1)
    for y in range(GRID_H+1):
        py=y*CELL_PIXELS
        cv2.line(img,(0,py),(WINDOW_W,py),(220,220,220),1)
    for x_map,y_map in CURRENT_SCAN_POINTS:
        cv2.circle(img,map_mm_to_pixel(x_map,y_map),3,(0,0,255),-1)
    if raw_path is not None:
        for i in range(len(raw_path)-1):
            cv2.line(img,node_to_pixel(*raw_path[i]),node_to_pixel(*raw_path[i+1]),(200,170,60),2)
    if final_path is not None:
        for i in range(len(final_path)-1):
            cv2.line(img,node_to_pixel(*final_path[i]),node_to_pixel(*final_path[i+1]),(255,0,0),4)
    cv2.circle(img,node_to_pixel(*start),8,(0,200,0),-1)
    cv2.circle(img,node_to_pixel(*goal),8,(255,0,255),-1)
    lidar_pos = BOTTOM_RIGHT_LIDAR_POS if CURRENT_START_ZONE==START_ZONE_BOTTOM_RIGHT else TOP_RIGHT_LIDAR_POS
    cv2.circle(img,map_mm_to_pixel(*lidar_pos),7,(0,0,0),-1)
    cv2.putText(img,f"Frames: {FRAME_COUNT}",(10,20),cv2.FONT_HERSHEY_SIMPLEX,0.5,(0,0,0),1)
    cv2.putText(img,f"Points: {len(CURRENT_SCAN_POINTS)}",(10,40),cv2.FONT_HERSHEY_SIMPLEX,0.5,(0,0,0),1)
    return img
# ============================================================
# 24. 主程序
# ============================================================
if __name__ == "__main__":
    build_known_map()
    start_node=get_start_node()
    try:
        ser=serial.Serial("COM13",230400,timeout=5)
        print("N10串口打开成功：COM13")
    except Exception as e:
        print("COM13打开失败：",e)
        raise SystemExit
    raw_path=None
    final_path=None
    last_planning_grid=None
    last_start_zone=CURRENT_START_ZONE
    last_goal=TARGET_NODE
    while True:
        try:
            frame=read_n10_frame(ser)
            if frame is not None:
                speed,start_angle,distances,end_angle=frame
                process_n10_frame(speed,start_angle,distances,end_angle)
                rebuild_lidar_grid()
                rebuild_planning_grid()
                need_replan=False
                if last_planning_grid is None:
                    need_replan=True
                elif not np.array_equal(last_planning_grid,planning_grid):
                    need_replan=True
                if CURRENT_START_ZONE!=last_start_zone:
                    need_replan=True
                if TARGET_NODE!=last_goal:
                    need_replan=True
                if need_replan:
                    raw_path,final_path=plan_path(start_node,TARGET_NODE,planning_grid)
                    last_planning_grid=planning_grid.copy()
                    last_start_zone=CURRENT_START_ZONE
                    last_goal=TARGET_NODE
            img=draw_map(raw_path,final_path,start_node,TARGET_NODE)
            cv2.imshow("N10 Realtime Map",img)
            key=cv2.waitKey(1)&0xFF
            if key==ord("1"):
                CURRENT_START_ZONE=START_ZONE_BOTTOM_RIGHT
                CURRENT_SCAN_POINTS.clear()
                LAST_START_ANGLE=None
                build_known_map()
                start_node=get_start_node()
            elif key==ord("2"):
                CURRENT_START_ZONE=START_ZONE_TOP_RIGHT
                CURRENT_SCAN_POINTS.clear()
                LAST_START_ANGLE=None
                build_known_map()
                start_node=get_start_node()
            elif key==ord("q") or key==27:
                break
        except Exception as e:
            print("运行错误：",e)
    ser.close()
    cv2.destroyAllWindows()
