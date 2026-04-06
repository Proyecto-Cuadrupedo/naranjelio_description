import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import math

class TrotGait(Node):
    def __init__(self):
        super().__init__('trot_gait')
        self.pub = self.create_publisher(
            JointTrajectory,
            '/joint_trajectory_controller/joint_trajectory', 10)
        self.timer = self.create_timer(0.02, self.step)
        self.t = 0.0

        self.joints = [
            'FemurBackDer', 'FemurBackIzq', 'FemurFrontDer', 'FemurFrontIzq',
            'FemurTibiaBackDer', 'FemurTibiaBackIzq', 'FemurTibiaFrontDer', 'FemurTibiaFrontIzq',
            'HipBackDer', 'HipBackIzq', 'HipFrontDer', 'HipFrontIzq',
        ]

        # Postura elevada confirmada
        self.stand = {
            'FemurBackDer':       -0.45,
            'FemurBackIzq':        0.45,
            'FemurFrontDer':       0.45,
            'FemurFrontIzq':      -0.45,
            'FemurTibiaBackDer':   1.0,
            'FemurTibiaBackIzq':  -1.0,
            'FemurTibiaFrontDer': -1.0,
            'FemurTibiaFrontIzq':  1.0,
            'HipBackDer':  0.0, 'HipBackIzq':  0.0,
            'HipFrontDer': 0.0, 'HipFrontIzq': 0.0,
        }

        self.femur_sign = {
            'FemurBackDer':  +1, 'FemurBackIzq':  -1,
            'FemurFrontDer': -1, 'FemurFrontIzq': +1,
        }
        self.tibia_sign = {
            'FemurTibiaBackDer':  -1, 'FemurTibiaBackIzq':  +1,
            'FemurTibiaFrontDer': +1, 'FemurTibiaFrontIzq': -1,
        }

        # Trot diagonal 50/50
        self.leg_phase = {
            'FrontIzq': 0.0,
            'BackDer':  0.0,
            'FrontDer': math.pi,
            'BackIzq':  math.pi,
        }

        def get_leg(j):
            for k in self.leg_phase:
                if k in j:
                    return k
            return None
        self.get_leg = get_leg

        self.f          = 0.7
        self.femur_amp  = 0.80
        self.tibia_lift = 0.75
        self.swing_frac = 0.50

        self.get_logger().info('Trot elevado corregido iniciado')

    def spot_gait(self, phi):
        """
        femur_val siempre en [-1, +1] donde:
          +1 = maximo adelante (inicio stance)
          -1 = maximo atras (fin stance)
        El signo de femur_sign se encarga de la direccion real.
        
        La fase de LIFT arranca desde donde termino el stance (-1)
        y va hacia neutro (0) mientras recoge la tibia.
        """
        p = phi % (2 * math.pi)
        swing_end = self.swing_frac * 2 * math.pi
        s1 = swing_end / 3.0    # fin lift
        s2 = swing_end * 2.0 / 3.0  # fin advance

        if p < s1:
            # LIFT: parte de -1 (fin stance), va a 0
            t = p / s1           # 0->1
            tibia_val = math.sin(t * math.pi / 2)      # 0->1
            femur_val = -math.cos(t * math.pi / 2)     # -1->0 suave

        elif p < s2:
            # ADVANCE: femur va de 0 a +1, tibia al maximo
            t = (p - s1) / (s2 - s1)   # 0->1
            tibia_val = 1.0
            femur_val = math.sin(t * math.pi / 2)      # 0->1

        elif p < swing_end:
            # PLANT: femur en +1, tibia baja para aterrizar
            t = (p - s2) / (swing_end - s2)  # 0->1
            tibia_val = math.cos(t * math.pi / 2)      # 1->0
            femur_val = 1.0

        else:
            # STANCE: femur de +1 a -1 lineal, tibia extendida
            t = (p - swing_end) / (2 * math.pi - swing_end)  # 0->1
            tibia_val = 0.0
            femur_val = 1.0 - 2.0 * t   # +1 -> -1

        return femur_val, tibia_val

    def step(self):
        self.t += 0.02
        phi = 2 * math.pi * self.f * self.t

        positions = []
        for j in self.joints:
            base = self.stand[j]
            leg  = self.get_leg(j)
            p    = phi + self.leg_phase[leg]
            femur_val, tibia_val = self.spot_gait(p)

            if j.startswith('FemurTibia'):
                pos = base + self.tibia_sign[j] * self.tibia_lift * tibia_val

            elif j.startswith('Femur'):
                amp = self.femur_amp if 'Back' in j else self.femur_amp * 0.85
                pos = base + self.femur_sign[j] * amp * femur_val

            else:
                pos = base

            positions.append(pos)

        msg = JointTrajectory()
        msg.joint_names = self.joints
        pt = JointTrajectoryPoint()
        pt.positions = positions
        pt.time_from_start.nanosec = 40_000_000
        msg.points = [pt]
        self.pub.publish(msg)

def main():
    rclpy.init()
    rclpy.spin(TrotGait())

if __name__ == '__main__':
    main()
