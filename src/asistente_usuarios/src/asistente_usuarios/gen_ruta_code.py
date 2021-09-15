#!/usr/bin/env python

import rospy
import actionlib
import time
from tf.transformations import quaternion_from_euler
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

# Puntos para generar la ruta
PUNTOS_RUTA = [
    [(0.5,0.0,0.0),(0.0,0.0,-90.0)], # Parte 1
    [(0.5,-1.0,0.0),(0.0,0.0,0.0)],
    [(1.0,-1.0,0.0),(0.0,0.0,90.0)],
    [(1.0,1.0,0.0),(0.0,0.0,180.0)],
    [(0.5,1.0,0.0),(0.0,0.0,-90.0)],
    [(0.5,0.0,0.0),(0.0,0.0,0.0)],   # Parte 2
    [(0.8,0.0,0.0),(0.0,0.0,90.0)],
    [(0.8,1.0,0.0),(0.0,0.0,0.0)],
    [(1.0,1.0,0.0),(0.0,0.0,-90.0)],
    [(1.0,-1.0,0.0),(0.0,0.0,180.0)],
    [(0.5,-1.0,0.0),(0.0,0.0,90.0)], 
    [(0.5,0.7,0.0),(0.0,0.0,0.0)],   # Parte 3
    [(1.0,0.7,0.0),(0.0,0.0,-90.0)],
    [(1.0,-0.7,0.0),(0.0,0.0,180.0)],
    [(0.5,-0.7,0.0),(0.0,0.0,90.0)],
    [(0.5,1.0,0.0),(0.0,0.0,0.0)],   # Parte 4
    [(0.7,1.0,0.0),(0.0,0.0,-90.0)],
    [(0.7,-1.0,0.0),(0.0,0.0,180.0)],
    [(0.5,-1.0,0.0),(0.0,0.0,90.0)],
    [(0.5,0.0,0.0),(0.0,0.0,0.0)]
]


class Ruta(object):
    def __init__(self):
        # Creamos un cliente para para el servidor de ActionLib del nodo "move_base"
        self.client_mvbase = actionlib.SimpleActionClient('move_base',MoveBaseAction)

        # Esperamos a que se inicie el servidor de ActionLib del nodo "move_base"
        self.client_mvbase.wait_for_server()


    
    # Función para preparar el mensaje "MoveBaseGoal" de cada punto de la ruta a seguir
    # Dentro del nodo de navegación cada punto será un objetivo
    def set_obj(self, punto):
        obj_data = MoveBaseGoal()
        obj_data.target_pose.header.frame_id="map"
        obj_data.target_pose.header.stamp = rospy.Time.now()
        obj_data.target_pose.pose.position.x = punto[0][0]
        obj_data.target_pose.pose.position.y = punto[0][1]
        obj_data.target_pose.pose.position.z = punto[0][2]

        # Conversión de ángulos de Euler a Quaternion
        x,y,z,w = quaternion_from_euler(punto[1][0], punto[1][1], punto[1][2])

        obj_data.target_pose.pose.orientation.x = x
        obj_data.target_pose.pose.orientation.y = y
        obj_data.target_pose.pose.orientation.z = z
        obj_data.target_pose.pose.orientation.w = w

        return obj_data


    # Función que recorre los diferentes puntos del recorrido, crea el mensaje correspondiente a un objetivo 
    # de navegación y lo envía al servidor ActionLib del nodo "move_base" 
    def ejec_ruta(self):
        for pt in PUNTOS_RUTA:
            obj_msg = self.set_obj(pt)
            self.client_mvbase.send_goal(obj_msg)
            self.client_mvbase.wait_for_result() # No procesamos el siguiente objetivo hasta que se cumpla el actual


    

    


