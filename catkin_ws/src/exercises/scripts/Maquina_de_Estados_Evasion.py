#!/usr/bin/env python3		

import math
import numpy as np
import rospy		#IMPORTAMOS LAS LIBRERIAS NECESARIAS
import time
import tf
import tf2_ros
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped 

import smach

import ros_numpy
from utils_evasion import *

############################################## FUNCIONES PARA LA LOCALIZACION ##############################################



def get_coords ():			#FUNCION PARA LOCALIZAR AL ROBOT.
    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer)
    trans = tfBuffer.lookup_transform('map', 'base_link', rospy.Time(),rospy.Duration(3))
    x_bot = trans.transform.translation.x
    y_bot = trans.transform.translation.y
    z_bot = trans.transform.translation.z
    return x_bot, y_bot, z_bot


############################################## FUNCIONES DEL ROBOT ##############################################

def wait_msg_meta_competencia():	#Definimos la funcion para esperar por el msg y obtener nuestros puntos de la meta.
	data_gps = rospy.wait_for_message('/meta_competencia', PoseStamped, timeout=10)
	x_gps = data_gps.pose.position.x
	y_gps = data_gps.pose.position.y
	z_gps = data_gps.pose.position.z
	return x_gps, y_gps, z_gps

def move_base_vel(vx, vy, vw):		#Envios de los comandos de velocidad
    twist = Twist()
    twist.linear.x = vx  			# Enviamos el valor de la velocidad en x (m/s)
    twist.linear.y = vy  			# Enviamos el valor de la velocidad en y (m/s)
    twist.angular.z = vw 			# Enviamos el valor de la rotacion en y (rad/s)
    base_vel_pub.publish(twist)  		#Con este comando publicamos al robot directamente(enviamos) 

def move_base(x,y,yaw,timeout=1): #Funcion para indicar cuanto tiempo mantendra las velocidades.
    start_time = rospy.Time.now().to_sec()
    while rospy.Time.now().to_sec() - start_time < timeout:  
        move_base_vel(x, y, yaw) #Mandamos llamar a la funcion anterior enviando 3 parametros (Vel.x, Vel.y, Rot.z)



def move_forward():	
    move_base(0.5,0,0)		

def move_backward():
    move_base(-0.5,0,0)

def turn_left():
    move_base(0,0,0.15708,10) 			# Las vueltas tendran una duracion de 10 seg para un angulo de 90º

def turn_right():
    move_base(0,0,-0.15708,10)

def get_lectura_cuant():
    try:
        lectura=np.asarray(laser.get_data().ranges)
        lectura=np.where(lectura>20,20,lectura) 	#REMUEVE INFINITOS

        right_scan=lectura[:300]
        left_scan=lectura[300:]
        ront_scan=lectura[300:360]

        sd,si=0,0
        if np.mean(left_scan)< 1.5: si=1		#DETECCIÓN. 
        if np.mean(right_scan)< 1.5: sd=1

    except:
        sd,si=0,0    

    return si,sd




##### Define state INITIAL #####

class S0(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome1','outcome2','outcome3','outcome4','outcome5'])
        self.counter = 0
        


    def execute(self,userdata):	# Lo que se ejecuta en cada estado va definido en la funcion execute 
        print('robot Estado S_0')        
        si,sd=get_lectura_cuant()	# Se toman las lecturas cuantizadas
        pose_robot = get_coords()


        if (math.isclose(pose_meta[0], pose_robot[0], abs_tol = 0.30)	
        and math.isclose(pose_meta[1], pose_robot[1], abs_tol = 0.30)):
        	return 'outcome5'
 
        if (si==0 and sd==0):
        	move_forward()
        	return 'outcome1'
        	       	            
        if (si==0 and sd==1): return 'outcome2'
        if (si==1 and sd==0): return 'outcome3'
        if (si==1 and sd==1): return 'outcome4'


class S1(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome1','outcome2'])
        self.counter = 0
        


    def execute(self,userdata):
    	# Aqui va lo que se desea ejecutar en el estado
        print('robot Estado S_1')

        #####Accion
        move_backward()
        return 'outcome1'
        

class S2(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome1','outcome2'])
        self.counter = 0
        


    def execute(self,userdata):
        print('robot Estado S_2')


        #####Accion
        turn_left()
        return 'outcome1'



class S3(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome1','outcome2'])
        self.counter = 0
        


    def execute(self,userdata):
        print('robot Estado S_3')

        #####Accion
        move_backward()
        return 'outcome1'




class S4(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome1','outcome2'])
        self.counter = 0
        


    def execute(self,userdata):
        print('robot Estado S_4')


        #####Accion
        turn_right()
        return 'outcome1' 



class S5(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome1','outcome2'])
        self.counter = 0
        


    def execute(self,userdata):
        print('robot Estado S_5')

        
        #####Accion
        move_backward()
        return 'outcome1' 




class S6(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome1','outcome2'])
        self.counter = 0
        

    def execute(self,userdata):
        print('robot Estado S_6')

        turn_right()  #CAMBIO
        return 'outcome1' 



class S7(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome1','outcome2'])
        self.counter = 0
        

    def execute(self,userdata):
        print('robot Estado S_7')

        #####Accion
        move_forward()
        return 'outcome1' 



class S8(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome1','outcome2'])
        self.counter = 0
        

    def execute(self,userdata):
        print('robot Estado S_8')

        #####Accion
        move_forward()
        return 'outcome1' 



class S9(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome1','outcome2'])
        self.counter = 0
        

    def execute(self,userdata):
        print('robot Estado S_9')

        #####Accion
        turn_right()
        return 'outcome1' 



class S10(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['outcome1','outcome2'])
        self.counter = 0
        

    def execute(self,userdata):
        pose_robot = get_coords()
        print('El robot ha llegado a la meta...!!!\n')			#MENSAJE DE FINALIZACIÓN.
        print(f'La meta era:\nX:{pose_meta[0]}\nY:{pose_meta[1]}\nLa posicion actual del robot es:\nX:{pose_robot[0]}\nY:{pose_robot[0]}')
        time.sleep(5)
        if (math.isclose(pose_meta[0], pose_robot[0], abs_tol = 0.30)
        and math.isclose(pose_meta[1], pose_robot[1], abs_tol = 0.30)): #SEGUIMOS COMPARANDO LOS PUNTOS EN CASO DE
        	return 'outcome1' 				      #QUE SE PUBLIQUE UNA META DISTINTA.
        else:
        	return 'outcome2'



def init(node_name):	
    global laser, base_vel_pub, pose_meta, pose_robot, tfBuffer, listener 		# Variables globales
    rospy.init_node(node_name)						#Iniciamos el nodo.
    base_vel_pub = rospy.Publisher('/hsrb/command_velocity', Twist, queue_size=10)  	#Aqui es donde publicamos.
    pose_robot_init = get_coords()
    pose_meta = wait_msg_meta_competencia()
    laser = Laser()



#Entry point
if __name__== '__main__':
    print("STATE MACHINE... \n EQUIPO: ELYSIUM ROBOTICS...")
    init("takeshi_smach")   
    sm = smach.StateMachine(outcomes = ["END"])     
    sm.userdata.sm_counter = 0
    sm.userdata.clear = False
    	
    with sm:
        #MAQUINA DE ESTADOS PARA EVASIÓN MODIFICADA.
        smach.StateMachine.add("s_0",   S0(),  transitions = {'outcome1':'s_0', 'outcome2':'s_1','outcome3':'s_3','outcome4':'s_5','outcome5':'s_10'})
        smach.StateMachine.add("s_1",   S1(),  transitions = {'outcome1':'s_2','outcome2':'s_10'})
        smach.StateMachine.add("s_2",   S2(),  transitions = {'outcome1':'s_0','outcome2':'s_10'})
        smach.StateMachine.add("s_3",   S3(),  transitions = {'outcome1':'s_4','outcome2':'s_10'})
        smach.StateMachine.add("s_4",   S4(),  transitions = {'outcome1':'s_0','outcome2':'s_10'})
        smach.StateMachine.add("s_5",   S5(),  transitions = {'outcome1':'s_6','outcome2':'s_10'})
        smach.StateMachine.add("s_6",   S6(),  transitions = {'outcome1':'s_7','outcome2':'s_10'})
        smach.StateMachine.add("s_7",   S7(),  transitions = {'outcome1':'s_8','outcome2':'s_10'})
        smach.StateMachine.add("s_8",   S8(),  transitions = {'outcome1':'s_9','outcome2':'s_10'})
        smach.StateMachine.add("s_9",   S9(),  transitions = {'outcome1':'s_0','outcome2':'s_10'})
        smach.StateMachine.add("s_10",  S10(), transitions = {'outcome1':'s_10','outcome2':'s_0'})


outcome = sm.execute()
