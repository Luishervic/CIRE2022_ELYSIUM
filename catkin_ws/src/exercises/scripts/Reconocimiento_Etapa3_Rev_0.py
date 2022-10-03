#!/usr/bin/env python3
# Importamos todas las librerias necesarias.
import tf
import os
import sys
import cv2
import time
import rospy
import smach
import ros_numpy
import numpy as np
from utils import * #---> Importamos todas las funcionalidades del paquete de Toyota
from gazebo_ros import gazebo_interface
from sensor_msgs.msg import LaserScan, PointCloud2
from geometry_msgs.msg import Pose, Quaternion ,TransformStamped


#####FUNCIONES DE RECONOCIMIENTO####

#Funcion principal con los argumentos de entrada de acuerdo al color de nuestros objetivos. :D

def Escaneo_Objetivos_Color(h_min, h_max):
    # Obtenemos la imagen
    image = rgbd.get_image()
    # Transformamos la imagen de RGB a BGR (Debido a criterios de compatibilidad de OpenCV)
    im_bgr = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
    # Obtenemos la imagen hsv que nos proporciona mayor robustez a los cambios de iluminacion.
    im_hsv = cv2.cvtColor(im_bgr, cv2.COLOR_BGR2HSV)
    # Delimitamos nuestra region de acuerdo al color del objetivo.
    region = (im_hsv > h_min) & (im_hsv < h_max)
    # Extraemos los puntos de la region.
    idx, idy = np.where(region[:,:,0] )
    # Creamos una nueva imagen y le asignamos los puntos obtenidos de la extraccion.
    mask = np.zeros((480,640))
    mask[idx,idy] = 255
    # Operaciones morfologicas de imagen.
    kernel = np.ones((5, 5), np.uint8)
    # Importante en caso de que nuestra imagen generada tenga huecos
    eroded_mask = cv2.erode(mask, kernel)
    dilated_mask = cv2.dilate(eroded_mask, kernel)
    # Cerramos los contornos abiertos de la imagen, está sera entonces la imagen que mandaremos para localizar los objetos
    kernel_1 = np.ones((10, 10), np.uint8)
    mask_closing = cv2.morphologyEx(dilated_mask, cv2.MORPH_CLOSE, kernel_1)
    return Localizacion_Objetos(mask_closing)

# Funcion que se ejecuta por defecto, para obtener, transformar y publicar las coordenadas.
def Localizacion_Objetos(imagen):
    # Obtenemos la informacion de la nube de puntos
    points= rgbd.get_points()
    # Buscamos los contornos de la imagen creada.
    contours, hierarchy = cv2.findContours(imagen.astype('uint8'), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    # Hacemos las operaciones con los contornos detectados para extraer ubicacion.
    # Obtenemos los momentos.
    mu = [None]*len(contours)
    br = [None]*len(contours)
    for i in range(len(contours)):
        exec("mi_contorno_%d = []" % (i))
        mu[i] = cv2.moments(contours[i]) 
        br[i]= cv2.boundingRect(contours[i])
    
    for j in range(0,len(contours)):
        for jy in range(br[j][0], br[j][0] + br[j][2]):
            for ix in range(br[j][1], br[j][1] + br[j][3]):
                aux=(np.asarray((points['x'][ix,jy],points['y'][ix,jy],points['z'][ix,jy])))
                if np.isnan(aux[0]) or np.isnan(aux[1]) or np.isnan(aux[2]):
                    'reject point'
                else:
                    exec("mi_contorno_%d.append(aux)" % (j))
           
        exec("xyz_%d = np.asarray(mi_contorno_%d)" % (j,j))
        exec("cent_%d = xyz_%d.mean(axis=0)" % (j,j))
        exec("x_%d, y_%d, z_%d = cent_%d" % (j,j,j,j))
        exec("broadcaster.sendTransform((x_%d,y_%d,z_%d),(0,0,0,1), rospy.Time.now(), 'Objeto_%d','head_rgbd_sensor_link')" % (j,j,j,j))
        # Esperamos 1 seg. para que las operaciones anteriores se efectuen caso contrario nos arrojara un error de extrapolacion.
        time.sleep(1)

    # Obtenemos las coordenadas respecto al mapa.
    Objeto_map_0 = listener.lookupTransform('map','Objeto_0', rospy.Time(0))
    Objeto_map_1 = listener.lookupTransform('map','Objeto_1', rospy.Time(0))
    return Objeto_map_0, Objeto_map_1


# Funcion para imprimir las coordenadas y transformadas absolutas de los objetos.
def Publicacion_Coordenadas(coord_0, coord_1):
    # Imprimimos las coordenadas en pantalla.
    print(f'Coordenadas del Objeto 0 con respecto del mapa son:\n x:{coord_0[0][0]}\n y:{coord_0[0][1]}\n z:{coord_0[0][2]}')
    print(f'Coordenadas del Objeto 1 con respecto del mapa son:\n x:{coord_1[0][0]}\n y:{coord_1[0][1]}\n z:{coord_1[0][2]}')
    # Publicamos las TF pero ahora con respecto del mapa.
    broadcaster.sendTransform((coord_0[0][0],coord_0[0][1],coord_0[0][2],),(0,0,0,1), rospy.Time.now(), 'Objeto_0_Map_Ref','map' )
    broadcaster.sendTransform((coord_1[0][0],coord_1[0][1],coord_1[0][2],),(0,0,0,1), rospy.Time.now(), 'Objeto_1_Map_Ref','map' )
    time.sleep(10)
    return 0



##### MAQUINA DE ESTADOS #####

class S0(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome1'])
        self.counter = 0

    def execute(self,userdata):
    	# Ejecutamos las acciones del estado 0
        print('Robot Estado S_0')
        move_arm_init()
        # Especificamos el punto objetivo en el orden (X, Y, Angulo)
        move_base_goal(3, 5.5, 0)
        # Movemos la persepcion para tener una deteccion clara.
        move_head_tilt(-0.8)
        return 'outcome1'
    

class S1(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome1','outcome2'])
        self.counter = 0
        
    def execute(self,userdata):
    	# Comenzamos con las acciones de reconocimiento.
        print('Robot Estado S_1')
        # Accion
        Coord_obj_0, Coord_obj_1 = Escaneo_Objetivos_Color(0,255)
        Publicacion_Coordenadas(Coord_obj_0, Coord_obj_1)
        return 'outcome1'


class S2(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome1','outcome2'])
        self.counter = 0
        
    def execute(self,userdata):
        print('robot Estado S_2')
        # Accion (X, Y, Angulo)
        move_base_goal(-2, 4, 180)
        # Apuntamos a los objetivos.
        move_head_tilt(-0.8)
        return 'outcome1'


class S3(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome1','outcome2'])
        self.counter = 0
        
    def execute(self,userdata):
        print('Robot Estado S_3')
        # Accion
        Coord_obj_0, Coord_obj_1 = Escaneo_Objetivos_Color(0,255)
        Publicacion_Coordenadas(Coord_obj_0, Coord_obj_1)
        return 'outcome1'


class S4(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome1','outcome2'])
        self.counter = 0
        
    def execute(self,userdata):
        print('Robot Estado S_4')
        #Accion (X, Y, Angulo)
        move_base_goal(0, 0.5, 90)
        return 'outcome1' 


class S5(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome1','outcome2'])
        self.counter = 0

    def execute(self,userdata):
        print('Robot Estado S_5')
        Coord_obj_0, Coord_obj_1 = Escaneo_Objetivos_Color(0,255)
        Publicacion_Coordenadas(Coord_obj_0, Coord_obj_1)
        return 'outcome1' 


class S6(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome1','outcome2'])
        self.counter = 0
    
    def execute(self,userdata):
        print('robot Estado S_6')
        #Accion
        move_base_goal(0, 0, 0)
        print('Rutina finalizada...!!!')
        return 'outcome1' 

def main():
    global listener, broadcaster, rgbd
    listener = tf.TransformListener()
    broadcaster= tf.TransformBroadcaster()
    # Llamamos a la funcion RGBD (Proveniente de utils).
    rgbd = RGBD()


if __name__ == '__main__':
    try:
        # Iniciamos el nodo.
        print('Ejercicio de Reconocimiento, Equipo: ---> Elysium...!!!')
        rospy.init_node('reconocimiento_e3')
        sm = smach.StateMachine(outcomes = ['END'])     
        sm.userdata.sm_counter = 0
        sm.userdata.clear = False
        main()
    except rospy.ROSInterruptException:
        rospy.loginfo("Test finalizado.")

    with sm:
        #Maquina de estados.
        smach.StateMachine.add("s_0",   S0(),  transitions = {'outcome1':'s_1'})
        smach.StateMachine.add("s_1",   S1(),  transitions = {'outcome1':'s_2','outcome2':'END'})
        smach.StateMachine.add("s_2",   S2(),  transitions = {'outcome1':'s_3','outcome2':'END'})
        smach.StateMachine.add("s_3",   S3(),  transitions = {'outcome1':'s_4','outcome2':'END'})
        smach.StateMachine.add("s_4",   S4(),  transitions = {'outcome1':'s_5','outcome2':'END'})
        smach.StateMachine.add("s_5",   S5(),  transitions = {'outcome1':'s_6','outcome2':'END'})
        smach.StateMachine.add("s_6",   S6(),  transitions = {'outcome1':'s_6','outcome2':'END'})
outcome = sm.execute()
