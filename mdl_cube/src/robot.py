# 逆運動学は毎回解かない
# 初回のみ解き, 解いた結果を保存しておく

# 3次元用のコードで書いておく
# 2次元はy-z平面

# A-satar内では
# pos_EE1, pos_EE2 (相対位置), ブロックの把持状態
# -> enable_actions (可能なアクション) を返す

# 逆運動学では
# pos_EE_root, pos_EE_targetで解く関数を作る

# 参考記事
# https://qiita.com/Conny_Brown_jp/items/4433ae9c70e8d60ec084#4-6%E8%87%AA%E7%94%B1%E5%BA%A6%E3%82%A2%E3%83%BC%E3%83%A0%E3%81%AE%E5%B9%BE%E4%BD%95%E8%A7%A3%E6%B3%95%E5%AE%9F%E8%B7%B5

# from module import ModuleBlock

from enum import Enum
import numpy as np
import math as math
# import random as random
import matplotlib.pyplot as plt
from matplotlib.widgets import Slider


# import sys
# from logging import getLogger, DEBUG, INFO, StreamHandler
# logger_level = DEBUG
# logger = getLogger(__name__)
# logger.setLevel(logger_level)
# handler = StreamHandler(sys.stderr)
# handler.setLevel(logger_level)
# logger.addHandler(handler)

from logging import getLogger, basicConfig, DEBUG, INFO
# import coloredlogs
# coloredlogs.install(level='DEBUG')
logger = getLogger(('app'))
basicConfig(
    level=DEBUG, filename='logger.log',
    filemode='w', format='%(asctime)s-%(process)s-%(levelname)s-%(message)s'
)


class RobotStatus(Enum):
    FREE = 0  # どこにも結合していない
    EE1_CONNECTING = 1  # EE1が結合している
    EE2_CONNECTING = 2  # 他のModuleに結合されている
    EE_BOTH_CONNECTING = 3  # EE1, EE2が結合している


class Robot:

    def __init__(self, id):
        self.id = id
        self.status = RobotStatus.FREE

        # 6DOF arm on 2D plane
        # module = ModuleBlock(0)
        # self.m_size = module.module_size
        self.m_size = np.array([1, 1])
        self.L = np.array([
            self.m_size[0]/2,
            self.m_size[0]/2,
            self.m_size[0] * np.sqrt(2),
            self.m_size[0] * np.sqrt(2)/2,
            self.m_size[0] * np.sqrt(2)/2,
            self.m_size[0]/2,
            self.m_size[0]/2])
        self.joint_state = np.array([0, 0, 0, 0, 0, 0])
        self.joint_pos = np.zeros((7, 3))

        self.color = "black"
        # self.connector_size = module.connector_size
        # self.dir = module.dir

    def change_status(self, status):
        self.status = status

    def foward_kinematics_6DOF(self, L, joint_state):
        """caltulate forward kinematics of 6DOF arm

        Args:
            L (list of float): リンク長*7
            joint_state (list of float): 関節角度*6

        Returns:
            Total_Transformation_Matrix: 同時変換行列
        """

        # 紙面手前方向:x
        # 紙面右方向:y
        # 紙面上方向:z
        Ts = []
        c0 = math.cos(joint_state[0])
        s0 = math.sin(joint_state[0])
        c1 = math.cos(joint_state[1])
        s1 = math.sin(joint_state[1])
        c2 = math.cos(joint_state[2])
        s2 = math.sin(joint_state[2])
        c3 = math.cos(joint_state[3])
        s3 = math.sin(joint_state[3])
        c4 = math.cos(joint_state[4])
        s4 = math.sin(joint_state[4])
        c5 = math.cos(joint_state[5])
        s5 = math.sin(joint_state[5])

        # 同時変換行列
        Ts.append(np.matrix([
            [c0, -s0, 0, 0],
            [s0, c0, 0, 0],
            [0, 0, 1, L[0]],
            [0, 0, 0, 1]]))
        Ts.append(np.matrix([
            [1, 0, 0, 0],
            [0, c1, -s1, 0],
            [0, s1, c1, L[1]],
            [0, 0, 0, 1]]))
        Ts.append(np.matrix([
            [1, 0, 0, 0],
            [0, c2, -s2, 0],
            [0, s2, c2, L[2]],
            [0, 0, 0, 1]]))
        Ts.append(np.matrix([
            [c3, -s3, 0, 0],
            [s3, c3, 0, 0],
            [0, 0, 1, L[3]],
            [0, 0, 0, 1]]))
        Ts.append(np.matrix([
            [1, 0, 0, 0],
            [0, c4, -s4, 0],
            [0, s4, c4, L[4]],
            [0, 0, 0, 1]]))
        Ts.append(np.matrix([
            [c5, -s5, 0, 0],
            [s5, c5, 0, 0],
            [0, 0, 1, L[5]],
            [0, 0, 0, 1]]))
        Ts.append(np.matrix([
            [1, 0, 0, 0],
            [0, 1, 0, 0],
            [0, 0, 1, L[6]],
            [0, 0, 0, 1]]))

        TTs = [
            Ts[0],
            Ts[0]*Ts[1],
            Ts[0]*Ts[1]*Ts[2],
            Ts[0]*Ts[1]*Ts[2]*Ts[3],
            Ts[0]*Ts[1]*Ts[2]*Ts[3]*Ts[4],
            Ts[0]*Ts[1]*Ts[2]*Ts[3]*Ts[4]*Ts[5],
            Ts[0]*Ts[1]*Ts[2]*Ts[3]*Ts[4]*Ts[5]*Ts[6]]
        return TTs

    def get_all_joint_pos(self, L, joint_state):
        """各関節位置を取得

        Args:
            L (list of float): リンク長*7
            joint_state (list of float): 関節角度*6 __

        Returns:
            joint_pos (list of float, round.3): 関節位置*6
        """
        TTs = self.foward_kinematics_6DOF(L, joint_state)
        joint_pos_FK = np.zeros((7, 3))
        for i in range(7):
            joint_pos_FK[i][0] = TTs[i][0, 3]
            joint_pos_FK[i][1] = TTs[i][1, 3]
            joint_pos_FK[i][2] = TTs[i][2, 3]
        return joint_pos_FK
        # return np.round(self.joint_pos, 3)  # 小数点第3位以下を四捨五入

    def get_EE_quaternion_from_root(self, joint_state):
        Theta = joint_state
        s53_0 = math.sin(Theta[5]/2 + Theta[3]/2 - Theta[0]/2)
        c53_0 = math.cos(Theta[5]/2 + Theta[3]/2 - Theta[0]/2)
        s5_3_0 = math.sin(Theta[5]/2 - Theta[3]/2 - Theta[0]/2)
        c5_3_0 = math.cos(Theta[5]/2 - Theta[3]/2 - Theta[0]/2)
        s3_0_5 = math.sin(Theta[3]/2 - Theta[0]/2 - Theta[5]/2)
        c3_0_5 = math.cos(Theta[3]/2 - Theta[0]/2 - Theta[5]/2)
        s530 = math.sin(Theta[5]/2 + Theta[3]/2 + Theta[0]/2)
        c530 = math.cos(Theta[5]/2 + Theta[3]/2 + Theta[0]/2)
        s12 = math.sin(Theta[1]/2 + Theta[2]/2)
        c12 = math.cos(Theta[1]/2 + Theta[2]/2)
        s4 = math.sin(Theta[4]/2)
        c4 = math.cos(Theta[4]/2)

        q_EE = np.zeros(4)
        q_EE[0] = c53_0*c4*s12 + c5_3_0*s4*c12
        q_EE[1] = -s53_0*c4*s12 - s5_3_0*s4*c12
        q_EE[2] = s3_0_5*s4*s12 + s530*c4*c12
        q_EE[3] = -c3_0_5*s4*s12 + c530*c4*c12

        return q_EE

    def get_quaternion_from_axis_theta(self, axis, theta):
        # x, y, z, w
        quaternion = np.zeros(4)
        quaternion[0] = axis[0] * math.sin(theta/2)
        quaternion[1] = axis[1] * math.sin(theta/2)
        quaternion[2] = axis[2] * math.sin(theta/2)
        quaternion[3] = math.cos(theta/2)
        return quaternion

    def multiply_quaternions(self, q1, q2):
        q_out = np.zeros(4)
        q_out[0] = q1[3]*q2[0] - q1[2]*q2[1] + q1[1]*q2[2] + q1[0]*q2[3]
        q_out[1] = q1[2]*q2[0] + q1[3]*q2[1] - q1[0]*q2[2] + q1[1]*q2[3]
        q_out[2] = -q1[1]*q2[0] + q1[0]*q2[1] + q1[3]*q2[2] + q1[2]*q2[3]
        q_out[3] = -q1[0]*q2[0] - q1[1]*q2[1] - q1[2]*q2[2] + q1[3]*q2[3]
        return q_out

    def euler_to_quaternion(self, roll, pitch, yaw):
        "オイラー角からクオータニオンを計算"
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)

        qx = sr * cp * cy - cr * sp * sy
        qy = cr * sp * cy + sr * cp * sy
        qz = cr * cp * sy - sr * sp * cy
        qw = cr * cp * cy + sr * sp * sy

        return np.array([qx, qy, qz, qw])

    def quaternion_to_euler(self, q):
        "クオータニオンからオイラー角を計算"
        # roll (x-axis rotation)
        sinr_cosp = 2 * (q[3] * q[0] + q[1] * q[2])
        cosr_cosp = 1 - 2 * (q[0] * q[0] + q[1] * q[1])
        roll = math.atan2(sinr_cosp, cosr_cosp)

        # pitch (y-axis rotation)
        sinp = 2 * (q[3] * q[1] - q[2] * q[0])
        if math.fabs(sinp) >= 1:
            # use 90 degrees if out of range
            pitch = math.copysign(math.pi / 2, sinp)
        else:
            pitch = math.asin(sinp)

        # yaw (z-axis rotation)
        siny_cosp = 2 * (q[3] * q[2] + q[0] * q[1])
        cosy_cosp = 1 - 2 * (q[1] * q[1] + q[2] * q[2])
        yaw = math.atan2(siny_cosp, cosy_cosp)

        return roll, pitch, yaw

    def adjust_angle_range(self, joint_state):
        # 回転角度の範囲を-π~πに修正
        joint_state_fixed = joint_state
        for j in range(len(joint_state)):
            if joint_state_fixed[j] > math.pi:
                joint_state_fixed[j] = joint_state_fixed[j] - \
                    int((joint_state_fixed[j] + math.pi)/(2*math.pi))*2*math.pi
            elif joint_state_fixed[j] < -math.pi:
                joint_state_fixed[j] = joint_state_fixed[j] - \
                    int((joint_state_fixed[j]-math.pi)/(2*math.pi))*2*math.pi
        return joint_state_fixed

    def inverse_kinematics_6DOF(self, L, target_pos_rel, target_q_rel):
        logger.debug("inverse_kinematics_6DOF")
        # 座標は全て相対座標
        logger.debug(f"target_pos_rel: {target_pos_rel}")
        logger.debug(f"target_q_rel: {target_q_rel}")
        Theta = np.zeros(6)
        for i in range(6):
            Theta[i] = None
        q = target_q_rel  # 根元基準でのクオータニオン
        joint_pos = np.zeros((7, 3))

        # joint_pos[1]
        joint_pos[0] = [0, 0, L[0]]
        joint_pos[1] = [0, 0, L[0]+L[1]]
        logger.debug(f"joint_pos[0]: {joint_pos[0]}")
        logger.debug(f"joint_pos[1]: {joint_pos[1]}")

        # joint_pos[4]
        vector_4_to_6 = np.zeros(3)
        vector_4_to_6[0] = 2*(q[0]*q[2] + q[3]*q[1])
        vector_4_to_6[1] = 2*(q[1]*q[2] - q[3]*q[0])
        vector_4_to_6[2] = q[3]**2 - q[0]**2 - q[1]**2 + q[2]**2
        # joint_pos[4] = target_pos_rel - (L[5]+L[6]) * vector_4_to_6
        joint_pos[4][0] = target_pos_rel[0] - (L[5]+L[6]) * vector_4_to_6[0]
        joint_pos[4][1] = target_pos_rel[1] - (L[5]+L[6]) * vector_4_to_6[1]
        joint_pos[4][2] = target_pos_rel[2] - (L[5]+L[6]) * vector_4_to_6[2]
        logger.debug(f"joint_pos[4]: {joint_pos[4]}")

        # Theta[0]
        # Theta[3]の値に関わらず, xy平面上のjoint_pos[4]から求められる
        # 特異点の処理:
        # joint_0とjoint_4, 2つのyaw軸が同一軸上にあるとき
        # Theta[0] = 0 としておく（不連続になることに注意）
        if joint_pos[4][0] == 0 and joint_pos[4][1] == 0:
            Theta[0] = 0
        else:
            # y軸の方向をTheta[0]=0とする
            Theta[0] = math.atan2(-joint_pos[4][0], joint_pos[4][1])
        # TODO: Theta[0]かTheta[1]を動かすかを現在の値によって変えたい
        # if abs(Theta[0]-self.joint_state[0]) \
        #         > abs(Theta[0]+math.pi-self.joint_state[0]):
        #     Theta[0] = Theta[0] + math.pi
        # elif abs(Theta[0]-self.joint_state[0]) \
        #         > abs(Theta[0]-math.pi-self.joint_state[0]):
        #     Theta[0] = Theta[0] - math.pi

        # -m.pi/2 < Theta[0] <= m.pi/2 に制限
        if Theta[0] > math.pi/2:
            Theta[0] = Theta[0] - math.pi
        elif Theta[0] <= -math.pi/2:
            Theta[0] = Theta[0] + math.pi
        logger.debug(f"Theta[0]: {Theta[0]}")

        # Theta[2]
        L_1_to_4_2 = (joint_pos[4][0] - joint_pos[1][0])**2\
            + (joint_pos[4][1] - joint_pos[1][1])**2 \
            + (joint_pos[4][2] - joint_pos[1][2])**2
        D = (L_1_to_4_2-(L[2]**2 + (L[3]+L[4])**2))/(2*L[2]*(L[3]+L[4]))
        Theta[2] = math.atan2(-math.sqrt(1-D**2), D)
        logger.debug(f"Theta[2]: {Theta[2]}")
        # y-z平面の象限によって場合分け
        if joint_pos[4][1] > 0 and joint_pos[4][0] > 0:
            Theta[2] = math.atan2(-math.sqrt(1-D**2), D)
        elif joint_pos[4][1] >= 0 and joint_pos[4][0] <= 0:
            Theta[2] = math.atan2(-math.sqrt(1-D**2), D)
        else:
            Theta[2] = math.atan2(math.sqrt(1-D**2), D)
        logger.debug(f"Theta[2]: {Theta[2]}")

        # Theta[1]
        Theta_1 \
            = math.atan2(joint_pos[4][2]-joint_pos[1][2],
                         math.sqrt((joint_pos[4][0]-joint_pos[1][0])**2
                                   + (joint_pos[4][1]-joint_pos[1][1])**2)) \
            - math.atan2((L[3]+L[4])*math.sin(Theta[2]),
                         L[2] + (L[3]+L[4])*math.cos(Theta[2]))
        Theta[1] = Theta_1 - math.pi/2
        logger.debug(f"Theta[1]: {Theta[1]}")

        # joint_pos[2]
        Ts_2 = self.foward_kinematics_6DOF(
            L, [Theta[0], Theta[1], 0, 0, 0, 0])
        # 平行ベクトル部分である, 0-3行目, 3列目を取り出す
        joint_pos[2][0] = Ts_2[2][0, 3]
        joint_pos[2][1] = Ts_2[2][1, 3]
        joint_pos[2][2] = Ts_2[2][2, 3]
        logger.debug(f"joint_pos[2]: {joint_pos[2]}")

        # Theta[4]
        L_2_to_6_2 = \
            (target_pos_rel[0]-joint_pos[2][0])**2 \
            + (target_pos_rel[1]-joint_pos[2][1])**2\
            + (target_pos_rel[2]-joint_pos[2][2])**2
        D_2 = -(((L[3]+L[4])**2 + (L[5]+L[6])**2)-L_2_to_6_2) \
            / (2*(L[3]+L[4])*(L[5]+L[6]))
        logger.debug(f'L[3]+L[4]: {L[3]+L[4]}')
        logger.debug(f'L[5]+L[6]: {L[5]+L[6]}')
        logger.debug(f'L_2_to_6_2: {L_2_to_6_2}')
        logger.debug(f"D_2: {D_2}")
        # まずは Theta[4]<0 と仮定して解く（Theta[3]が1周するとき）
        Theta[4] = math.acos(D_2)
        logger.debug(f"Theta[4]: {Theta[4]}")

        # Theta[3]
        # calculate q of each joint
        q_of_each_joint = np.zeros((5, 4))
        q_of_each_joint[0] = self.get_quaternion_from_axis_theta(
            [0, 0, 1], Theta[0])
        q_of_each_joint[1] = self.get_quaternion_from_axis_theta(
            [1, 0, 0], Theta[1])
        q_of_each_joint[2] = self.get_quaternion_from_axis_theta(
            [1, 0, 0], Theta[2])
        # q0~1 -> q1
        q_0to1 = self.multiply_quaternions(
            q_of_each_joint[0], q_of_each_joint[1])
        # q0~2 -> q2
        q_0to2 = self.multiply_quaternions(
            q_0to1, q_of_each_joint[2])
        C4 = math.cos(Theta[4])
        S4 = math.sin(Theta[4])
        A = np.matrix([
            [2*(q_0to2[2]*q_0to2[3] - q_0to2[0]*q_0to2[1])*S4,
             (q_0to2[0]**2 - q_0to2[1]**2 - q_0to2[2]**2 + q_0to2[3]**2)*S4],
            [(q_0to2[0]**2 - q_0to2[1]**2 + q_0to2[2]**2 - q_0to2[3]**2)*S4,
             2*(q_0to2[3]*q_0to2[2] + q_0to2[0]*q_0to2[1])*S4]])
        B = np.matrix([
            [vector_4_to_6[0]-2*(q_0to2[2]*q_0to2[0]+q_0to2[3]*q_0to2[1])*C4],
            [vector_4_to_6[1]-2*(q_0to2[1]*q_0to2[2]-q_0to2[0]*q_0to2[3])*C4]])
        A_inv = np.linalg.inv(A)
        # A * XY = B の解を得る
        XY = A_inv * B
        Theta[3] = math.atan2(XY[1, 0], XY[0, 0])

        logger.debug(f"Theta[3]: {Theta[3]}")
        # TODO: 現在に近い方に合わせる
        # if abs(Theta[3]-self.joint_state[3]) \
        #         > abs(Theta[3]+math.pi-self.joint_state[3]):
        #     Theta[3] = Theta[3] + math.pi
        # elif abs(Theta[3]-self.joint_state[3]) \
        #         > abs(Theta[3]-math.pi-self.joint_state[3]):
        #     Theta[3] = Theta[3] - math.pi
        if Theta[3] >= math.pi/2:
            Theta[3] = Theta[3] - math.pi
        elif Theta[3] < -math.pi/2:
            Theta[3] = Theta[3] + math.pi
        logger.debug(f"Theta[3]: {Theta[3]}")

        # Theta[5]
        # calculate q of each joint
        q_of_each_joint[3] = self.get_quaternion_from_axis_theta(
            [0, 0, 1], Theta[3])
        q_of_each_joint[4] = self.get_quaternion_from_axis_theta(
            [1, 0, 0], Theta[4])

        # q0~4 -> q4
        q_0to3 = self.multiply_quaternions(
            q_0to2, q_of_each_joint[3])
        q_0to4 = self.multiply_quaternions(
            q_0to3, q_of_each_joint[4])
        C5 = (q_0to4[0]*q[0] + q_0to4[1]*q[1]) / (q_0to4[0]**2 + q_0to4[1]**2)
        S5 = (q[0]*q_0to4[1] - q[1]*q_0to4[0]) / (q_0to4[0]**2 + q_0to4[1]**2)
        Theta[5] = 2*math.atan2(S5, C5)
        # TODO
        if abs(Theta[5]-self.joint_state[5]) \
                > abs(Theta[5]+math.pi*2-self.joint_state[5]):
            Theta[5] = Theta[5] + math.pi*2
        elif abs(Theta[5]-self.joint_state[5]) \
                > abs(Theta[5]-math.pi*2-self.joint_state[5]):
            Theta[5] = Theta[5] - math.pi*2
        logger.debug(f"Theta[5]: {Theta[5]}")

        joint_state = self.adjust_angle_range(Theta)

        # 計算が合っているか確認
        # joint_2_4_6 三角形が2パターンある
        EE_pos_IK = self.get_all_joint_pos(self.L, joint_state)[-1]
        EE_q_from_root_IK = self.get_EE_quaternion_from_root(joint_state)
        if not np.allclose(target_pos_rel, EE_pos_IK, atol=1e-2):
            flag_change_theta_4 = True
            logger.info("joint_pos NG")
        elif not np.allclose(target_q_rel, EE_q_from_root_IK, atol=1e-2):
            flag_change_theta_4 = True
            logger.info("EE_q NG")
        else:
            flag_change_theta_4 = False
            logger.info("joint_pos and EE_q OK")

        if flag_change_theta_4:
            logger.debug("change_theta_4")
            # if q[0]*q_EE[0] < 0:  # 正なら同じ向き
            Theta[4] = -Theta[4]
            logger.debug(f"Theta[4]: {Theta[4]}")
            # Theta[4]以降を計算しなおす
            # Theta[3]
            # calculate q of each joint
            q_of_each_joint = np.zeros((5, 4))
            q_of_each_joint[0] = self.get_quaternion_from_axis_theta(
                [0, 0, 1], Theta[0])
            q_of_each_joint[1] = self.get_quaternion_from_axis_theta(
                [1, 0, 0], Theta[1])
            q_of_each_joint[2] = self.get_quaternion_from_axis_theta(
                [1, 0, 0], Theta[2])
            # q0~1 -> q1
            q_0to1 = self.multiply_quaternions(
                q_of_each_joint[0], q_of_each_joint[1])
            # q0~2 -> q2
            q_0to2 = self.multiply_quaternions(
                q_0to1, q_of_each_joint[2])
            C4 = math.cos(Theta[4])
            S4 = math.sin(Theta[4])
            A = np.matrix([
                [2*(q_0to2[2]*q_0to2[3] - q_0to2[0]*q_0to2[1])*S4,
                 (q_0to2[0]**2 - q_0to2[1]**2
                  - q_0to2[2]**2 + q_0to2[3]**2)*S4],
                [(q_0to2[0]**2 - q_0to2[1]**2
                  + q_0to2[2]**2 - q_0to2[3]**2)*S4,
                 2*(q_0to2[3]*q_0to2[2] + q_0to2[0]*q_0to2[1])*S4]])
            B = np.matrix([
                [vector_4_to_6[0]-2*(q_0to2[2]*q_0to2[0] +
                                     q_0to2[3]*q_0to2[1])*C4],
                [vector_4_to_6[1]-2*(q_0to2[1]*q_0to2[2] -
                                     q_0to2[0]*q_0to2[3])*C4]])
            A_inv = np.linalg.inv(A)
            # A * XY = B の解を得る
            XY = A_inv * B
            Theta[3] = math.atan2(XY[1, 0], XY[0, 0])
            logger.debug(f"Theta[3]: {Theta[3]}")

            # Theta[5]
            # calculate q of each joint
            q_of_each_joint[3] = self.get_quaternion_from_axis_theta(
                [0, 0, 1], Theta[3])
            q_of_each_joint[4] = self.get_quaternion_from_axis_theta(
                [1, 0, 0], Theta[4])

            # q0~4 -> q4
            q_0to3 = self.multiply_quaternions(
                q_0to2, q_of_each_joint[3])
            q_0to4 = self.multiply_quaternions(
                q_0to3, q_of_each_joint[4])
            C5 = (q_0to4[0]*q[0] + q_0to4[1]*q[1]) / \
                (q_0to4[0]**2 + q_0to4[1]**2)
            S5 = (q[0]*q_0to4[1] - q[1]*q_0to4[0]) / \
                (q_0to4[0]**2 + q_0to4[1]**2)
            Theta[5] = 2*math.atan2(S5, C5)
            logger.debug(f"Theta[5]: {Theta[5]}")

            joint_state = self.adjust_angle_range(Theta)

        return joint_state

    def visualize_robot(self, L, joint_state):
        joint_pos = self.get_all_joint_pos(L, joint_state)
        fig = plt.figure()
        plt.title('Robot')
        ax = fig.add_subplot(111, projection='3d')

        plt.axis('auto')
        plt.xlim([-2, 2])
        plt.ylim([-2, 2])
        ax.set_zlim([-1, 2])

        plt.grid()
        graph, = ax.plot(
            joint_pos[:, 0], joint_pos[:, 1], joint_pos[:, 2])
        graph.set_linestyle('-')
        graph.set_marker('o')
        graph.set_markersize(10)
        plt.grid()
        plt.show()


"""
# def check_inversed_kinematics(self):
#     L = self.L
#     joint_state_random = np.zeros(6)
#     for i in range(6):
#         if i == 0:
#             joint_state_random[i] = 0
#         elif i == 1:
#             joint_state_random[i] = random.uniform(
#                 -math.pi / 2, 0)
#         elif i == 2:
#             joint_state_random[i] = random.uniform(-math.pi/2, math.pi/2)
#         elif i == 3:
#             joint_state_random[i] = 0
#         elif i == 4:
#             joint_state_random[i] = random.uniform(-math.pi/2, math.pi/2)
#         elif i == 5:
#             joint_state_random[i] = 0
#     joint_pos_FK_random = self.get_all_joint_pos(L, joint_state_random)
#     if joint_pos_FK_random[4][1] > 0 and joint_pos_FK_random[4][0] > 0:
#         if joint_state_random[2] > 0:
#             joint_state_random[2] = -joint_state_random[2]
#             joint_state_random[1] = atan
#     elif joint_pos_FK_random[4][1] >= 0 and joint_pos_FK_random[4][0] <= 0:
#         if joint_state_random[2] > 0:
#             joint_state_random[2] = -joint_state_random[2]
#     else:
#         if joint_state_random[2] < 0:
#             joint_state_random[2] = -joint_state_random[2]

#     EE_q_random = self.get_EE_quaternion(joint_state_random)
#     logger.info(f"joint_state_random:\n{joint_state_random}")
#     # logger.debug(f"joint_state_random:\n{joint_state_random}")
#     logger.debug(f"joint_pos_FK_random:\n{joint_pos_FK_random}")

#     joint_state_IK_random = self.inverse_kinematics_6DOF(
#         L, joint_pos_FK_random[-1], EE_q_random)
#     joint_pos_IK_random = self.get_all_joint_pos(L, joint_state_IK_random)
#     logger.debug(f"joint_state_IK_random:\n{joint_state_IK_random}")
#     logger.debug(f"joint_pos_IK_random:\n{joint_pos_IK_random}")

#     # 小数点以下第2位までの比較
#     if np.allclose(joint_state_random, joint_state_IK_random, atol=1e-2):
#         logger.debug("joint_state OK")
#         if np.allclose(
#                 joint_pos_FK_random, joint_pos_IK_random, atol=1e-2):
#             logger.debug("joint_pos OK")
#         return True
#     else:
#         logger.debug("joint_state NG")
#         return False

# def check_inversed_kinematics(self):
#     L = self.L

#     EE_pos_random = np.zeros(3)
#     EE_pos_random[0] = random.uniform(0, 1)
#     EE_pos_random[1] = random.uniform(0, 1)  # -1~1 OK
#     EE_pos_random[2] = random.uniform(0, 1)
#     EE_roll_random = random.uniform(-math.pi/2, 0)
#     EE_pitch_random = 0
#     EE_yaw_random = 0
#     logger.info(f"EE_pos_random:\n{EE_pos_random}")
#     logger.info(f"EE_roll_random:\n{EE_roll_random}")
#     logger.info(f"EE_pitch_random:\n{EE_pitch_random}")
#     logger.info(f"EE_yaw_random:\n{EE_yaw_random}")

#     EE_q_random = self.euler_to_quaternion(EE_roll_random,
#                                            EE_pitch_random, EE_yaw_random)

#     joint_state_IK_random = self.inverse_kinematics_6DOF(
#         L, EE_pos_random, EE_q_random)

#     joint_pos_IK_random = self.get_all_joint_pos(L, joint_state_IK_random)
#     EE_pos_IK_random = joint_pos_IK_random[-1]
#     logger.debug(f"joint_state_IK_random:\n{joint_state_IK_random}")
#     logger.debug(f"joint_pos_IK_random:\n{joint_pos_IK_random}")

#     # 小数点以下第2位までの比較
#     if np.allclose(
#             EE_pos_IK_random, EE_pos_random, atol=1e-2):
#         logger.debug("joint_pos OK")
#         return True
#     return False
"""


if __name__ == "__main__":
    robot = Robot(0)
    # logger.debug('test inverse kinematics')
    # if robot.check_inversed_kinematics():
    #     logger.info("check_inversed_kinematics OK")
    # else:
    #     logger.info("check_inversed_kinematics NG")
    # logger.debug('test inversed kinematics finished')

    joint_state_FK = np.array(
        [-math.pi/4,  # -math.pi/2 <= joint_state[0] < math.pi/2
         -math.pi/4,   # -math.pi < joint_state[1] < math.pi
         -math.pi/2,   # -math.pi < joint_state[2] < math.pi
         0,   # -math.pi/2 < joint_state[3] < math.pi/2
         math.pi/4,   # -math.pi < joint_state[4] < math.pi
         math.pi*1/4    # -math.pi < joint_state[5] < math.pi
         ])

    logger.info(f"joint_state_FK:\n{joint_state_FK}")
    joint_pos_FK = robot.get_all_joint_pos(robot.L, joint_state_FK)
    robot_EE_q_from_root = robot.get_EE_quaternion_from_root(joint_state_FK)
    logger.debug(f"joint_pos_FK:\n{joint_pos_FK}")
    joint_state_IK = robot.inverse_kinematics_6DOF(
        robot.L, joint_pos_FK[-1], robot_EE_q_from_root)
    logger.debug(f"joint_state_IK:\n{joint_state_IK}")
    joint_pos_IK = robot.get_all_joint_pos(robot.L, joint_state_IK)
    logger.debug(f"joint_pos_IK:\n{joint_pos_IK}")

    # 小数点以下第2位までの比較
    if np.allclose(joint_state_FK, joint_state_IK, atol=1e-2):
        logger.info("joint_state OK")
        if np.allclose(joint_pos_FK, joint_pos_IK, atol=1e-2):
            logger.info("joint_pos OK")
        else:
            logger.info("joint_pos NG")
    else:
        logger.info("joint_state NG")

    robot.visualize_robot(robot.L, joint_state_FK)
