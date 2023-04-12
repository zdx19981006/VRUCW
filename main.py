import math


class Agent:
    def __init__(self):
        self.ego = None  # 本车
        self.vehicle_list = []  # 所有车辆的集合
        self.human_list = []  # 所有行人的集合
        self.t1 = 1  # 驾驶员平均反应时间
        self.t1min = 0  # 驾驶员最小反应时间
        self.t1max = 0  # 驾驶员最大反应时间
        self.t2 = 1  # 汽车反应时间
        self.ego_amax = 1  # 本车最大制动减速度
        self.ego_usual_amax = 1  # 本车正常制动减速度
        self.vechile_bmax = 1  # 目标车辆的最大制动减速度
        self.ttc = 1  # 碰撞预警时间
        self.pedMinSpeed = 1  # 目标对象最小速度
        self.pedMaxSpeed = 1  # 目标对象最大速度
        self.vehLength = 1  # 半车长

    def vrucw(self):
        egoInfo = self.ego
        vehInfo = self.vehicle_list
        humInfo = self.human_list

        # 当前车辆信息
        egoLoc = self.get_location(egoInfo)  # 本车位置
        ego_yaw = self.get_rotation(egoInfo)[1]  # 本车航向角
        ego_veloc = self.get_velocity(egoInfo)[0]  # 本车当前速度
        ego_x_veloc = self.get_velocity(egoInfo)[1]  # 本车当前横向速度
        ego_y_veloc = self.get_velocity(egoInfo)[2]  # 本车当前纵向速度
        ego_X_acc = self.get_acceleration(egoInfo)[1]  # 当前车辆横向加速度
        ego_y_acc = self.get_acceleration(egoInfo)[2]  # 当前车辆纵向加速度
        brake_dis = (self.t1 + self.t2) * ego_veloc + (ego_veloc * ego_veloc / (2 * self.ego_amax))  # 制动距离
        dfmin = brake_dis + self.t1min * ego_veloc  # 最小纵向距离
        dfmax = (self.t1max + self.ttc) * ego_veloc  # 最大纵向距离
        dsmin = self.pedMinSpeed * (dfmin / ego_veloc)  # 最小横向距离
        dsmax = self.pedMaxSpeed * (dfmax / ego_veloc)  # 最大横向距离

        # 低速参与者
        for human in humInfo:
            humLoc = self.get_location(humInfo)
            # 安全范围
            D = [egoLoc[0] + (dfmin + self.vehLength) * math.sin(ego_yaw) + dsmin * math.cos(ego_yaw),
                 egoLoc[1] + (dfmin + self.vehLength) * math.cos(ego_yaw) - dsmin * math.sin(ego_yaw)]
            C = [egoLoc[0] + (dfmin + self.vehLength) * math.sin(ego_yaw) - dsmin * math.cos(ego_yaw),
                 egoLoc[1] + (dfmin + self.vehLength) * math.cos(ego_yaw) + dsmin * math.sin(ego_yaw)]
            A = [egoLoc[0] + (dfmax + self.vehLength) * math.sin(ego_yaw) - dsmax * math.cos(ego_yaw),
                 egoLoc[1] + (dfmax + self.vehLength) * math.cos(ego_yaw) + dsmax * math.sin(ego_yaw)]
            B = [egoLoc[0] + (dfmax + self.vehLength) * math.sin(ego_yaw) + dsmax * math.cos(ego_yaw),
                 egoLoc[1] + (dfmax + self.vehLength) * math.cos(ego_yaw) - dsmax * math.sin(ego_yaw)]

            if self.get_isInScope(humLoc, A, B, C, D):
                return True

        # 高速
        for vehicle in vehInfo:
            vehicle_Loc = self.get_location(vehicle)  # 目标车辆的位置
            vehicle_x_speed = self.get_velocity(vehicle)[1]  # 目标车辆横向速度
            vehicle_y_speed = self.get_velocity(vehicle)[2]  # 目标车辆纵向速度
            vehicle_x_acc=self.get_acceleration[1] #目标车辆横向加速度
            vehicle_y_acc=self.get_acceleration[2] #目标车辆纵向加速度
            # vechile_bmax：目标车辆最大制动加速度  egoLoc：本车位置  ego_veloc：本车当前速度  t1平均反应时间
            # ego_usual_amax 当前车辆正常减速度
            #纵向安全距离
            d1 = ego_y_veloc * self.t1 + 0.5 * ego_y_acc * self.t1 * self.t1 + (ego_y_veloc + ego_y_acc * self.t1) * (
                        ego_y_veloc + ego_y_acc * self.t1) / (2 * self.ego_usual_amax)
            d2 = vehicle_y_speed * vehicle_y_speed / (2 * self.vechile_bmax)
            y_dmin = d1 - d2  # 纵向距离
            if y_dmin <= 0:
                return True


            #横向安全距离
            ego_vt_max=ego_x_veloc+ego_X_acc*self.t1 #反应时间内，本车的最大横向速度
            veh_vt_max=vehicle_x_speed+vehicle_x_acc*self.t1 #反应时间内 目标车辆的最大横向速度
            ego_vt_dis=((ego_x_veloc+ego_vt_max)/2)*self.t1+ego_vt_max*ego_vt_max/(2*self.ego_usual_amax) #速度减为0时本车行驶过的路程
            veh_vt_dis=((vehicle_x_speed+veh_vt_max)/2)*self.t1+veh_vt_max*veh_vt_max/(2*self.ego_usual_amax) #速度减为0时目标车辆行驶过的距离
            dis_lat=ego_vt_dis+veh_vt_dis+ego_x_veloc #横向安全距离
            if(dis_lat<=0):
                return True

        return True

    def get_location(self, actor):
        x = y = z = 0
        return [x, y, z]

    def get_rotation(self, actor):
        x = y = z = 0
        return [x, y, z]

    def get_velocity(self, actor):
        v = v_x = v_y = 0
        return [v, v_x, v_y]

    def get_isInScope(self, humLoc, A, B, C, D):
        pass

    def get_acceleration(self, actor):
        acc = acc_x = acc_y = 0
        return [acc, acc_x, acc_y]
