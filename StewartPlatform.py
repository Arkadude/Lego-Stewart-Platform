from pybricks.hubs import InventorHub
from pybricks.pupdevices import Motor, ColorSensor, UltrasonicSensor
from pybricks.parameters import Button, Color, Direction, Port, Side, Stop
from pybricks.robotics import DriveBase
from pybricks.tools import wait, StopWatch
import umath
from usys import stdin
from uselect import poll

#--------------------objects--------------------------------------------------
hub = InventorHub()
A=Motor(Port.A, reset_angle=False)
B=Motor(Port.B, reset_angle=False)
C=Motor(Port.C, reset_angle=False)
D=Motor(Port.D, reset_angle=False)
E=Motor(Port.E, reset_angle=False)
F=Motor(Port.F, reset_angle=False)

#registers standard input
keyboard=poll()
keyboard.register(stdin)

#--------------------error class----------------------------------------------
class PhysicalError(Exception):
    def __init__(self, message):
        self.message = message

#--------------------global variables-----------------------------------------
#geometric constants:
mr=10 #radius of LEGO 140 tooth gear in studs
mo=2 #motors are offset by 2 studs perpendicular to the radius
alpha=umath.atan2(2,10)
r=umath.sqrt(mr**2+mo**2)

#gear ratio
actuator_ratio=1920 #degrees per stud

#a dictionary to store all the initial positions of the actuator
#joints (12 total), represented in studs. 
jpos={
    'A0': [r*umath.sin(alpha),-r*umath.cos(alpha),0],
    'B0': [r*umath.cos(0.5234-alpha),r*umath.sin(0.5234-alpha),0],
    'C0': [r*umath.cos(0.5234+alpha),r*umath.sin(0.5234+alpha),0],
    'D0': [r*umath.cos(2.618-alpha),r*umath.sin(2.618-alpha),0],
    'E0': [r*umath.cos(2.618+alpha),r*umath.sin(2.618+alpha),0],
    'F0': [-r*umath.sin(alpha),-r*umath.cos(alpha),0],
    'A1': [r*umath.cos(5.7596-alpha),r*umath.sin(5.7596-alpha),0],
    'B1': [r*umath.cos(5.7596+alpha),r*umath.sin(5.7596+alpha),0],
    'C1': [r*umath.sin(alpha),r*umath.cos(alpha),0],
    'D1': [-r*umath.sin(alpha),r*umath.cos(alpha),0],
    'E1': [r*umath.cos(3.665-alpha),r*umath.sin(3.665-alpha),0],
    'F1': [r*umath.cos(3.665+alpha),r*umath.sin(3.665+alpha),0]    
}

#--------------------vector operations----------------------------------------
def vsum(u,v): #vector sum
    return([u[0]+v[0], u[1]+v[1], u[2]+v[2]])
def norm(u): #normalizes the vector u so that ||u||=1
    magnitude=umath.sqrt(u[0]**2+u[1]**2+u[2]**2)
    return([u[0]/magnitude, u[1]/magnitude,u[2]/magnitude])

def dist(u,v): #returns the distance between 2 points 
    magnitude=umath.sqrt((u[0]-v[0])**2+(u[1]-v[1])**2+(u[2]-v[2])**2)
    return magnitude
    


#--------------------quaternion operations------------------------------------
def qsum(p,q): #quaternion sum
    return([p[0]+q[0], p[1]+q[1], p[2]+q[2],p[3]+q[3]])

def qmult(p,q): #quaternion multiplication
    result=[0,0,0,0]
    result[0]=p[0]*q[0]-(p[1]*q[1]+p[2]*q[2]+p[3]*q[3]) #scalar component
    result[1]=p[0]*q[1]+q[0]*p[1]+p[2]*q[3]-p[3]*q[2] #i component
    result[2]=p[0]*q[2]+q[0]*p[2]+p[3]*q[1]-p[1]*q[3] #j component
    result[3]=p[0]*q[3]+q[0]*p[3]+p[1]*q[2]-p[2]*q[1] #k component
    return(result)

def qconj(p): #quaternion conjugate
    return([p[0],-p[1],-p[2],-p[3]])

def qLq(q,v): #performs the rotation operation qvq*
    return(qmult(qmult(q,v),qconj(q)))

def qrotate(v, theta, u): #rotates the vector v (3x1)
#through an angle theta (radians) about the vector axis u (3x1)
#||u||=1 is required

    #convert the vectors to quaternions
    q_v=[0,v[0],v[1],v[2]]
    q=[umath.cos(theta/2), umath.sin(theta/2)*u[0],umath.sin(theta/2)*u[1],umath.sin(theta/2)*u[2]]
    result=qLq(q,q_v) #performs the rotation operation
    return(result[-3:])
#-----------------------------------------------------------------------------

#inverse kinematics of the stewart platform is calculated using quaternions
#the inputs are a translation vector u_t(3x1), an axis of rotation u_r(3x1)
#and the angle of rotation theta(degrees)
#returns a 6x1 list with required actuator lengths to reach desired position
def inverse_kinematics(u_t, u_r, theta):
    #normalize u_r:
    u_r=norm(u_r)
    #convert theta to radians
    theta=umath.radians(theta)

    a_lengths=[0,0,0,0,0,0]
    base_j_pos=[ #a 6x3 list with base joint positions
        jpos['A0'],
        jpos['B0'],
        jpos['C0'],
        jpos['D0'],
        jpos['E0'],
        jpos['F0']
    ] 
    target_j_pos=[ #a 6x3 list with each upper joint after the transformations
        jpos['A1'],
        jpos['B1'],
        jpos['C1'],
        jpos['D1'],
        jpos['E1'],
        jpos['F1']
    ] 

    for i in range(6):
        #apply rotation
        target_j_pos[i]= qrotate(target_j_pos[i],theta, u_r)
        #apply translation
        target_j_pos[i]= vsum(target_j_pos[i], u_t)
        #calculate actuator lengths
        a_lengths[i]=dist(target_j_pos[i], base_j_pos[i])
    
    #check if the lengths are within the physical limits of the actuators
    #if not, raise PhysicalError
    max_length=max(a_lengths)
    min_length=min(a_lengths)
    print('tryng to move to: '+str(a_lengths)+' [studs]')
    if max_length>17 or min_length<12:
        raise PhysicalError('required actuator lengths are out of physical range of the model: '+str(a_lengths))
    
    return a_lengths

#-----------------------------motion commands---------------------------------    
def stop_motion(): #holds all motors at once
    A.stop()
    B.stop()
    C.stop()
    D.stop()
    E.stop()
    F.stop()

def homing(): #home the motors until stalled
    speed=400 #degrees/second
    duty_limit=30 #percent
    A.run_until_stalled(-speed, then=Stop.COAST, duty_limit=duty_limit)
    B.run_until_stalled(-speed, then=Stop.COAST, duty_limit=duty_limit)
    C.run_until_stalled(-speed, then=Stop.COAST, duty_limit=duty_limit)
    D.run_until_stalled(-speed, then=Stop.COAST, duty_limit=duty_limit)
    E.run_until_stalled(-speed, then=Stop.COAST, duty_limit=duty_limit)
    F.run_until_stalled(-speed, then=Stop.COAST, duty_limit=duty_limit)

    #reset absolute positions
    A.reset_angle(12*actuator_ratio)
    B.reset_angle(12*actuator_ratio)
    C.reset_angle(12*actuator_ratio)
    D.reset_angle(12*actuator_ratio)
    E.reset_angle(12*actuator_ratio)
    F.reset_angle(12*actuator_ratio)

def move_to(M, target, speed): #move actuator to target position in studs
    M.run_target(speed,target*actuator_ratio, then=Stop.HOLD, wait=False)

def get_pos(M): #gets the absolute position of the motor in studs
    return (M.angle()/actuator_ratio)

def get_current_positions(): #gets the absolute position of all motors in studs
    result=[
        get_pos(A),
        get_pos(B),
        get_pos(C),
        get_pos(D),
        get_pos(E),
        get_pos(F)
    ]
    return result

def group_move(targets, wait_for_completion=True):
    distances=[0,0,0,0,0,0]
    speeds=[0,0,0,0,0,0]
    current_positions=get_current_positions()
    max_speed=1200 #degrees per second
    min_speed=100 #degrees per second
    #calculate relative motor speeds:
    for i in range(6):
        distances[i]=abs(targets[i]-current_positions[i])
    target_time=max(distances)/max_speed
    for i in range(6):
        speeds[i]=max([distances[i]/target_time,min_speed])
    
    #perform motions:
    move_to(A,targets[0],speeds[0])
    move_to(B,targets[1],speeds[1])
    move_to(C,targets[2],speeds[2])
    move_to(D,targets[3],speeds[3])
    move_to(E,targets[4],speeds[4])
    move_to(F,targets[5],speeds[5])

    #wait for motors to complete if required:
    if wait_for_completion: 
        while not (A.done() and B.done() and C.done() and D.done() and E.done() and F.done()):
            wait(10)

def readIMU(): #reads the hub imu
    n=5.0 #perform moving average over how many readings
    pitch, roll=0,0
    for i in range (n):
        p,r=hub.imu.tilt() #read hub tilt and roll
        #correct roll to account for hub being upside down
        if r>0:
            r=-r+179
        else:
            r=-r-180

        pitch=pitch+p
        roll=roll+r
        wait(50)
    
    #perform average
    pitch=-pitch/n
    roll=-roll/n


    print('current pitch: '+str(pitch))
    print('current roll: '+str(roll))
    return pitch, roll

#-----------------------------remote control----------------------------------
def read_keyboard():
    #if a key has been pressed:
    if keyboard.poll(0):
        #read the key
        key=stdin.read(1)
    else:
        key="none" 
    
    return str(key)

#control the motion of the platform using the numpad
def keyboard_control(): 
    target=inverse_kinematics([0,0,12],[1,0,0],0) #default position
    polling_delay=10 #time to wait between key polls [ms]
    max_angle=7 #maximum allowed tilt angle in degrees

    prev_key="none"
    key="none"
    while (key!="s"): #"s" stops the program

        #read new key 100 times until not none to avoid missed inputs
        for i in range(30):
            wait(polling_delay)
            key=read_keyboard()
            if key!='none':
                break
        print(key)

        #if key hasn't changed continue stall/motion

        #if new key was pressed update target 
        if key!=prev_key:
            #case machine
            if key=='5': #center
                target=inverse_kinematics([0,0,12],[1,0,0],0)
            elif key=='9': #back-left
                target=inverse_kinematics([0,0,12], [-1,1,0], -max_angle)
            elif key=='8': #back
                target=inverse_kinematics([0,0,12], [1,0,0], max_angle)
            elif key=='7': #back-right
                target=inverse_kinematics([0,0,12], [1,1,0], max_angle)
            elif key=='6': #left
                target=inverse_kinematics([0,0,12], [0,1,0], -max_angle)
            elif key=='4': #right
                target=inverse_kinematics([0,0,12], [0,1,0], max_angle)
            elif key=='3': #forward-left
                target=inverse_kinematics([0,0,12], [1,1,0], -max_angle)
            elif key=='2': #forward
                target=inverse_kinematics([0,0,12], [1,0,0], -max_angle)
            elif key=='3': #forward-right
                target=inverse_kinematics([0,0,12], [-1,1,0], max_angle)
            
            group_move(target, wait_for_completion=False)

        #execute motion if no key presses were recorded
        if key=='none' and prev_key=='none':
            stop_motion()
            
            

            #wait 
            wait(polling_delay)
        #update previous key
        prev_key=key

    print('Thanks for playing!')
#-----------------------------advanced----------------------------------------
def forward(t, angle): #tilts forward and waits a set amount (millisecond)
    target=inverse_kinematics([0,0,12], [1,0,0], -angle)
    group_move(target)
    wait(t)

def backward(t, angle): #tilts backward and waits a set amount (millisecond)
    target=inverse_kinematics([0,0,12], [1,0,0], angle)
    group_move(target)
    wait(t)

def left(t, angle): #tilts left and waits a set amount (millisecond)
    target=inverse_kinematics([0,0,12], [0,1,0], -angle)
    group_move(target)
    wait(t)

def right(t, angle): #tilts right and waits a set amount (millisecond)
    target=inverse_kinematics([0,0,12], [0,1,0], angle)
    group_move(target)
    wait(t)

def forward_left(t, angle): #tilts forward-left and waits a set amount (millisecond)
    target=inverse_kinematics([0,0,12], [1,1,0], -angle)
    group_move(target)
    wait(t)

def backward_right(t, angle): #tilts backward-right and waits a set amount (millisecond)
    target=inverse_kinematics([0,0,12], [1,1,0], angle)
    group_move(target)
    wait(t)

def forward_right(t, angle): #tilts forward-right and waits a set amount (millisecond)
    target=inverse_kinematics([0,0,12], [-1,1,0], angle)
    group_move(target)
    wait(t)

def backward_left(t, angle): #tilts backward-left and waits a set amount (millisecond)
    target=inverse_kinematics([0,0,12], [-1,1,0], -angle)
    group_move(target)
    wait(t)


def solve_maze(): #solves the current maze (scripted)
    right(500, 5)
    forward(500, 5)
    left(250, 4)
    backward(250, 4)
    left(500, 4)
    forward(250, 4)
    left(250, 4)
    forward(100, 4)
    right(100, 2)
    group_move(inverse_kinematics([0,0,12], [1,5,0], 10))
    right(250, 4)
    forward(250, 4)
    left(250, 4)
    forward(250, 4)
    right(250, 4)
    forward(500, 4)
    wait(2000)
    group_move(inverse_kinematics([0,0,12], [0,1,0], 0))

def level(): #levels the platform based on the hub IMU readings
    pitch_target=0
    roll_target=1.3
    #get the platform into a known position.
    #this is required because we haven't implemented direct kinematics
    group_move(inverse_kinematics([0,0,13], [0,1,0], 0))

    wait(1000)
    pitch, roll=readIMU() #read hub tilt and roll

    #calculate errors:
    pitch_error=pitch_target-pitch
    roll_error=roll_target-roll

    #if errors are 0 no need to correct:
    if pitch_error==0 and roll_error==0:
        print('Platform is already level, exiting')
        return

    #convert to radians:
    pitch_error=umath.radians(pitch_error)
    roll_error=umath.radians(roll_error)

    #calculate quaternions:
    u=[1,0,0] #pitch is a rotation around x axis
    v=[0,1,0] #roll is a rotation around y axis
    qpitch=[umath.cos(pitch_error/2), umath.sin(pitch_error/2)*u[0],umath.sin(pitch_error/2)*u[1],umath.sin(pitch_error/2)*u[2]]
    qroll=[umath.cos(roll_error/2), umath.sin(roll_error/2)*v[0],umath.sin(roll_error/2)*v[1],umath.sin(roll_error/2)*v[2]]
    #the imu order of rotation is first pitch then roll.
    #to reverse it, first we need to perform a roll, then a pitch
    q=qmult(qroll,qpitch)

    #from q we extract the angle and axis of correction
    correction_angle=umath.acos(q[0])*2
    correction_axis=[q[1]/umath.sin(correction_angle/2),
    q[2]/umath.sin(correction_angle/2),
    q[3]/umath.sin(correction_angle/2)]

    #convert the angle to degrees
    correction_angle=umath.degrees(correction_angle)

    print('To level the platform, need to rotate by: '+ str(correction_angle)+ 'degrees, around: '+str(correction_axis)+' axis')

    #perform correction:
    correction=inverse_kinematics([0,0,13], correction_axis, correction_angle)
    group_move(correction)

    #check the IMU again:
    wait(1000)
    readIMU()

def preview(): #previews the capabilities of the platform
    #roll
    group_move(inverse_kinematics([0,0,13], [0,1,0], 10))

    group_move(inverse_kinematics([0,0,13], [0,1,0], -10))
    wait(500)
    group_move(inverse_kinematics([0,0,13], [0,1,0], 0))
    wait(500)

    #pitch
    group_move(inverse_kinematics([0,0,13], [1,0,0], 10))
    wait(500)
    group_move(inverse_kinematics([0,0,13], [0,1,0], 0))
    wait(500)

     #yaw
    group_move(inverse_kinematics([0,0,13], [0,0,1], 18))
    wait(500)
    group_move(inverse_kinematics([0,0,13], [0,0,1], 0))
    wait(500)

    #xyz
    n=12
    for i in range (n):
        angle=umath.radians(360/n*i)
        x=3*umath.cos(angle)
        y=3*umath.sin(angle)
        group_move(inverse_kinematics([x,y,13], [0,0,1], 0))

    group_move(inverse_kinematics([0,0,15.5], [0,0,1], 0))
    group_move(inverse_kinematics([0,0,12], [0,0,1], 0))      
    
    #composite
    n=24
    for i in range (2*n):
        angle=umath.radians(360/n*i)
        x=2*umath.cos(angle)
        y=2*umath.sin(angle)
        group_move(inverse_kinematics([x,y,13], [-y,x,0], -7))
    #return
    group_move(inverse_kinematics([0,0,13], [0,0,1], 0)) 


#-------------------------Main------------------------------------------------
#homing()
#a=inverse_kinematics([0,0,13], [0,1,0], 0)
#group_move(a)
#preview()

#wait(1000)
#readIMU()
a=inverse_kinematics([0,0,12], [0,1,0], 0)
group_move(a)
#wait(1000)
#readIMU()
#while True:
#    readIMU()
#    wait(1000)
#level()

#solve_maze()
for i in range(100):
    read_keyboard()
    wait(10)
keyboard_control()
