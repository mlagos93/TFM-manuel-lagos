import rospy, math
import numpy as np
import dynamic_reconfigure.client
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class detecusuario(object):
    def __init__(self):
        self.pub = rospy.Publisher('cmd_vel', Twist, queue_size = 10)
        self.client = dynamic_reconfigure.client.Client('/move_base/DWAPlannerROS') # Vamos a usar este paquete para reconfigurar la velocidad durante la navegación

    
    # Función para detectar al usuario en un rango ajustado de modo que detecte las piernas del mismo. Usaremos el Lidar para dicha funcionalidad
    # La señal del Lidar vendrá filtrada, tarea de la que se encarga el nodo "detec_usuario_filtro"
    def detect(self):
        self.msg = rospy.wait_for_message("scan", LaserScan)
        self.rango_valores = []
        self.error = 0.0
        self.distancia_ideal = 0.3
        self.velocidad_ideal = 0.17
        self.proporcional = 0.0
        self.vel = 0.0
        self.distancia_min = 0.0


        # Vamos a sustituir valores infinitos o nan por 2.5 y 0 respectivamente
        for i in range (250,178,-1) + range(179,109,-1):
            if self.msg.ranges[i] == float('Inf'):
                self.rango_valores.append(3.5) 
            elif math.isnan(self.msg.ranges[i]):
                self.rango_valores.append(0) 
            else:
                self.rango_valores.append(self.msg.ranges[i])

        self.rango_valores = np.array(self.rango_valores)
        self.distancia_min = np.min(self.rango_valores[np.nonzero(self.rango_valores)]) # Distancia mínima descartando los 0

        print('distancia', self.distancia_min)

        # A partir de la distancia a la que se encuentre la persona y la distancia deseada, calculamos el error y lo utilizamos para enviar la velocidad al Robot
        # Ojo, rango del Laser (0.12 - 0.7)
        # Vamos a  tomar como distancia idonea del usuario: 0.30
        
        self.error = self.distancia_ideal - self.distancia_min

        self.proporcional = self.velocidad_ideal + (self.error*0.3)

        print('proporcional', self.proporcional)

        # Controlamos que no nos pasamos de la velocidad máxima del Robot ni permitimos velocidades negativas.
        
        if self.proporcional > 0.22:
            self.vel = 0.22
        elif self.proporcional < 0.0:
            self.vel = 0.0
        else:
            self.vel = self.proporcional

        print('sel vel', self.vel)

        return self.vel

    def ini_detec(self):
        while not rospy.is_shutdown():
            self.vel_aux = self.detect()
            param = {'max_vel_x' : self.vel_aux}

            self.client.update_configuration(param)