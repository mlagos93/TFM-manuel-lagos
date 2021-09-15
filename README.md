# TFM-manuel-lagos

Requisitos previos: 

1. Configurar e instalar los paquetes de TurtleBot3. (Sección 6.2 de la memoria)
2. Copiar el archivo "dwa_local_planner_params_burger.yaml" subido en directorio "archivos-configuracion" en  
   el directorio /opt/ros/melodic/share/turtlebot3_navigation/param de tu ordenador.
3. Generar el mapa del entorno. (Apartado SLAM de la sección 6.3 de la memoria. 
  

Los pasos a seguir para ejecutar la aplicación son:

$ roscore   (Ordenador)
$ roslaunch turtlebot3_bringup turtlebot3_robot.launch (TurtleBot3)
$ roslaunch turtlebot3_navigation turtlebot3_navigation.launch map_file:=$HOME/nombre-mapa.yaml. (Ordenador)
$ roslaunch detec_usuario_filtro detec_usuario_filtro.launch (Ordenador)
$ roslaunch detec_usuario detec_usuario.launch (Ordenador)
$ roslaunch asistente_usuarios iniciar_ejercicio.launch (Ordenador)
