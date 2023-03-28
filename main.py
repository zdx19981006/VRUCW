import math


class Agent:
    def __init__(self):
        self.ego = None # 本车
        self.vehicle_list = [] # 所有车辆的集合
        self.human_list = [] # 所有行人的集合
        self.t1 = 1  # 驾驶员平均反应时间
        self.t1min = 0  # 驾驶员最小反应时间
        self.t1max = 0  # 驾驶员最大反应时间
        self.t2 = 1  # 汽车反应时间
        self.amax = 1  # 最大制动减速度
        self.ttc=1 #碰撞预警时间
        self.pedMinSpeed=1 #目标对象最小速度
        self.pedMaxSpeed=1 #目标对象最大速度
        self.vehLength=1 #半车长
    def vrucw(self):
        egoInfo=self.ego
        vehInfo=self.vehicle_list
        humInfo=self.human_list

        #当前车辆信息
        egoLoc = self.get_location(egoInfo)  # 本车位置
        ego_yaw = self.get_rotation(egoInfo)[1]  # 本车航向角
        ego_veloc = self.get_velocity(egoInfo)  # 本车当前速度
        brake_dis = (self.t1 + self.t2) * ego_veloc + (ego_veloc * ego_veloc / (2 * self.amax))  # 制动距离
        dfmin = brake_dis + self.t1min * ego_veloc  # 最小纵向距离
        dfmax = (self.t1max + self.ttc) * ego_veloc  # 最大纵向距离
        dsmin = self.pedMinSpeed * (dfmin / ego_veloc)  # 最小横向距离
        dsmax = self.pedMaxSpeed * (dfmax / ego_veloc)  # 最大横向距离

        #低速参与者
        for human in humInfo:
            humLoc=self.get_location(humInfo)
            #安全范围
            D=[egoLoc[0]+(dfmin+self.vehLength)*math.sin(ego_yaw)+dsmin*math.cos(ego_yaw),egoLoc[1]+(dfmin+self.vehLength)*math.cos(ego_yaw)-dsmin*math.sin(ego_yaw)]
            C=[egoLoc[0]+(dfmin+self.vehLength)*math.sin(ego_yaw)-dsmin*math.cos(ego_yaw),egoLoc[1]+(dfmin+self.vehLength)*math.cos(ego_yaw)+dsmin*math.sin(ego_yaw)]
            A=[egoLoc[0]+(dfmax+self.vehLength)*math.sin(ego_yaw)-dsmax*math.cos(ego_yaw),egoLoc[1]+(dfmax+self.vehLength)*math.cos(ego_yaw)+dsmax*math.sin(ego_yaw)]
            B=[egoLoc[0]+(dfmax+self.vehLength)*math.sin(ego_yaw)+dsmax*math.cos(ego_yaw),egoLoc[1]+(dfmax+self.vehLength)*math.cos(ego_yaw)-dsmax*math.sin(ego_yaw)]

            if self.get_isInScope(humLoc,A,B,C,D):
                return True

        #高速
        for x in vehInfo :
            vehLoc=self.get_location(x)
            egoLoc=self.get_location(egoInfo)


        return True

    def get_location(self, actor):
        x = y = z = 0
        return [x, y, z]

    def get_rotation(self, actor):
        x = y = z = 0
        return [x, y, z]

    def get_velocity(self ,actor):
        v=0
        return v
    def get_isInScope(self,taget,A,B,C,D):
        return True




