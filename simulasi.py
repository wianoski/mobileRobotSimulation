"""
Simulasi Go to Goal
Kuliah EMBEDDED SYETEM ROBOTICS
DOSEN: Dr. Bayu Erfianto
Fakultas Informatika
TELKOM UNIVERSITY


========================================================================
DESKRIPSI:

Berikut adalah script Pyhton untuk mensimulasikan Differential Drive 
Mobile Robot Dengan Proportional Controller.

Simulasi ini untuk memberikan ilustrasi bagaimana
mengendalikan kecepatan (speed Controller) dari mobile robot.
Mobile robot akan bergerak menuju goal dengan kecepatan tertentu menggunakan
kendali proporsional. Dalam hal ini, kecepatan mobile robot adalah proporsional
terhadap jarak antara goal dengan mobile robot. Ketika mobile robot mendekati 
goal maka kecepatan mobile robot akan berkurang, dan akhirnya akan berhenti
saat berada di posisi goal (asumsi eror jarak mobile robot terhadap
posisi mendekati 0).

Bacalah petunjuk serta soal dalam script ini, apa yang ditanyakan dan apa yang
harus dikerjakan. Hasil dari simulasi anda HARUS DIBAWA saat UTS. Beberapa 
pertanyaan pada saat UTS akan berkaitan dengan hasil simulasi yang anda buat. 

========================================================================
"""

import matplotlib.pyplot as plt
import numpy as np
from pylab import figure,show
from random import random


'''
========================================================================
DEFINISI PARAMETER

rho adalah jarak antara vehicle dengan posisi goal.
alpha adalah orientasi (heading) posisi awal robot terhadap goal.
beta sudut heading robot terhadap heading dari goal.
    
Kp_rho, Kp_alpha dan Kp_beta adalah gain untuk mengendalikan robot menuju 
posisi goal agar sesuai dengan orientasi sudutnya.

Kp_beta digunakan khusus untuk mengendalikan rotasi robot agar sudut robot 
sesuai dengan sudut  akhir dari goal agar heading robot = heading goal.
========================================================================
'''

'''
========================================================================
KONSTANTA
Berikut adalah nilai konstanta untuk proportional controller.
Silakan disesuaikan ketika anda gunakan untuk simulasi.
========================================================================
'''
Kp_rho = 0.5 
Kp_alpha = 25
Kp_beta = -5

'''
========================================================================
SOAL UNTUK DISIMULASIKAN

PETUNJUK:
Ubahlah ketiga konstanta tersebut untuk melihat pengaruhnya terhadap 
trajektori mobile robot. Amati dan catat setiap hasilnya, kemudian print
hasil trajektori (Figure 1) dan kecepatan robot (Figure 2 ). Hasil plot
anda harus dibawa pada saat UTS.

1. Pada Kp_rho,Kp_alpha, Kp_beta berapa robot lebih cepat mencapai tujuan
   (Lihat Plot Kecepatan Robot), amati dan plot hasilnya.
2. Ubah Kp_beta bernilai positif, amati dan plot hasilnya.
3. Ubah Kp_alpha bernilai negatif, amati dan plot hasilnya.
4. Ubah Kp_rho bernilai negatif, amati dan plot hasilnya.
========================================================================
'''

'''
========================================================================
KONSTANTA DIFFERENTIAL DRIVE ROBOT

Konstanta Differential Drive Mobile Robot Mobile robot menggunakan 
penggerak differential wheel pada roda belakang dan castor wheel pada 
roda depan, sehingga bodi robot diasumsikan sebagai segitiga                 
========================================================================
'''
r = 2 # radius roda mobile robot dalam cm
L = 8 # Axis length dari body robot dalam cm
arena = 250 # arena simulasi adalah 250 cm x 250 cm

'''
========================================================================
SOAL UNTUK DISIMULASIKAN

PETUNJUK:
Ubahlah nilai r dan L untuk melihat pengaruhnya terhadap 
trajektori mobile robot. Amati dan catat setiap hasilnya, kemudian print
hasil trajektori (Figure 1) dan kecepatan robot (Figure 2 ). Hasil plot
anda harus dibawa pada saat UTS.

5. Ubah nilai radius roda menjadi >2, dan <2, amati dan plot hasilnya.
6. Ubah nilai axis length menjadi >8, dan <8, amati dan plot hasilnya.
========================================================================
'''



'''
========================================================================
KONSTANTA DAN PARAMETER LAINNYA
========================================================================
'''
dt = 0.1 # Sampling Time untuk simulasi.

error_jarak = 0.2 # eror jarak minimum posisi robot saat ini terhadap goal

show_animation1 = True 
show_animation2 = True 


'''
========================================================================
 Behavior utama untuk menggerakan robot dari initial position 
 / posisi awal robot menuju ke goal position. 
========================================================================
'''
def move_to_pose(x_start, y_start, theta_start, x_goal, y_goal, theta_goal):

    # x,y dan theta posisi awal dan sudut heading / orientasi robot 
    x = x_start
    y = y_start
    theta = theta_start

    xo = 100
    yo = 100


    # delta x dan y, yaitu jarak robot saat ini terhadap posisi goal
    delta_x = x_goal - x
    delta_y = y_goal - y

    delta_xo = xo - x
    delta_yo = yo - y

    # array untuk menyimpan informasi trajektori vehicle (x,y)
    x_traj, y_traj = [], []
    
    # array untuk menyimpan informasi kecepatan vehicle per 
    # unit waktu (tick). v_ adalah kecepatan translasi / linear
    # v_x adalah kecepatan linear pada sumbu x
    # v_y adalah kecepatan linear pada sumbu y
    # v_t adalah kecepatan linear mobile robot
    v_x,v_y,v_t, v,e_jarak,omega = [], [], [], [],[],[]
    
    # Menghitung Euclidean distance / jarak robot ke posisi goal
    rho = np.sqrt(delta_x**2 + delta_y**2)
    doa = np.sqrt(delta_xo**2 + delta_yo**2)
    
    # Jalankan simulasi selama jarak vehicle terhadap
    # posisi goal > error_jarak  

    # for error plot
    distance = []

    while rho > error_jarak:
        
        # update array yang berisi trajektori vehicle
        x_traj.append(x)
        y_traj.append(y)

        # update array yang berisi delta posisi vehicle saat ini terhadap goal
        # dalam sumbu x dan y
        delta_x = x_goal - x
        delta_y = y_goal - y

        delta_xo = xo - x
        delta_yo = yo - y


        # update Euclidean distance ke goal
        rho = np.sqrt(delta_x**2 + delta_y**2)
        doa = np.sqrt(delta_xo**2 + delta_yo**2)
        
        alpha = (np.arctan2(delta_y, delta_x) -
                 theta + np.pi) % (2 * np.pi) - np.pi
        beta = (theta_goal - theta - alpha + np.pi) % (2 * np.pi) - np.pi

        # kecepatan awal linear vehicle sebagai fungsi proporsional terhadap
        # rho atau terhadap eror / selisih jarak vehicle terhadap goal
        #v = r/2*(wR+wL) # Robot translasional velocity
        v = (r/2)*(Kp_rho * rho)
        
        # kecepatan sudut sebagai fungsi proporsional terhadap
        # alpha dan beta
        #w = r/L*(wR-wL) # Robot angular velocity
        w = (Kp_alpha * alpha + Kp_beta * beta)/L
        
        # hitung sudut vehicle terhadap sudut goal yang diinginkan
        if alpha > np.pi / 2 or alpha < -np.pi / 2:
            v = -v
        
        #update theta saat ini
        theta = theta + (w * dt)

        # update posisi vehicle saat ini pada sumbu x dan y
        x = x + (v * np.cos(theta) * dt)
        y = y + (v * np.sin(theta) * dt)
        
        #hitung kecepatan pada sumbu x dan y
        vx = v * np.cos(theta) * dt
        vy = v * np.sin(theta) * dt 
        
        vx = abs(vx)
        vy = abs(vy)
        #hitung kecepatan total
        vt = np.sqrt(vx**2 + vy**2)
        
        # update parameter dalam array
        v_x.append(vx)
        v_y.append(vy)
        v_t.append(vt)
        e_jarak.append(rho)
        omega.append(w)
        
        # untuk  enjalankan animasi pergerakan mobile robot
        if show_animation1:
            plt.cla()
            plt.figure(1)
            plt.title('Trajektori Pergerakan Vehicle')
            plt.xlabel('X (cm)')
            plt.ylabel('Y (cm)')
            
            #plot posisi & orientasi awal mobile robot berwarna merah
            plt.arrow(x_start, y_start, np.cos(theta_start),
                      np.sin(theta_start), color='r', width=2,)
            
            #plot posisi & orientasi awal mobile robot berwarna hijau
            plt.arrow(x_goal, y_goal, np.cos(theta_goal),
                      np.sin(theta_goal), color='g', width=2,)
            
            plt.text(200, 210, 'Posisi Goal')
            
            #memanggil fungsi plot vehicle beserta trajektorinya
            plot_vehicle(x, y, theta, x_traj, y_traj)
            

        if show_animation2:
            plt.cla()
            plt.figure(2)
            plot_velocity(v_t)

        #error plot
        deltaX = x_goal - x
        deltaY = y_goal - y

        nowDist = np.sqrt(deltaX**2 + deltaY**2)
        distance.append(nowDist)
        print("Distance now : x ",x," y ",y, " from ", nowDist)
        error_plot(distance)

        if rho < error_jarak:
            print("End of Simulation")
            print("Close Figure to Exit")

''' 
========================================================================
Fungsi untuk mem-plot vehicle beserta trajektorinya
Trajektori Vehicle ditampilkan pada Figure 1
========================================================================
'''
def plot_vehicle(x, y, theta, x_traj, y_traj):
    
    # parameter segitiga untuk membuat mobile robot
    p1_i = np.array([5, 0, 1]).T
    p2_i = np.array([-5, 3, 1]).T
    p3_i = np.array([-5, -3, 1]).T


    # Fungsi allignment dengan matriks rotasi agar frame 
    # mobile robot diproyeksikan ke frame global (X,Y)
    T = rotation_matrix(x, y, theta)
    p1 = np.matmul(T, p1_i)
    p2 = np.matmul(T, p2_i)
    p3 = np.matmul(T, p3_i)

    # plot vehicle / robot berupa segitiga dengan garis hitam
    plt.plot([p1[0], p2[0]], [p1[1], p2[1]], 'k-')
    plt.plot([p2[0], p3[0]], [p2[1], p3[1]], 'k-')
    plt.plot([p3[0], p1[0]], [p3[1], p1[1]], 'k-')

    # Plot trajektori vehicle / mobile robot
    plt.figure(1)
    plt.plot(x_traj, y_traj, 'b-.')
    
    # Area Simulasi
    plt.xlim(0, arena)
    plt.ylim(0, arena)
    plt.pause(dt) # refresh setiap dt

''' 
========================================================================
Fungsi untuk mem-plot kecepatan vehicle selama simulasi
Kecepatan vehicle diperlihatkan pada Figure 2
========================================================================
'''
def plot_velocity(v_t):
    plt.figure(2)
    
    # Kecepatan Mobile Robot
    plt.plot(v_t, 'r-') 
    plt.title('Kecepatan Translasi')
    plt.xlabel('Waktu')
    plt.pause(dt)


'''
========================================================================
SOAL UNTUK DISIMULASIKAN

7. Buatlah fungsi untuk untuk mem-plot error jarak, yaitu jarak dari posisi
saat ini dari mobile robot terhadap posisi goal. dan tampilkan hasil plot
pada Figure 3
========================================================================
'''


# Fungsi matriks rotasi / transformasi    
def rotation_matrix(x, y, theta):
    return np.array([
        [np.cos(theta), -np.sin(theta), x],
        [np.sin(theta), np.cos(theta), y],
        [0, 0, 1]
    ])

def error_plot(distance):
    plt.figure(3)
    plt.plot(distance, "-r")
    plt.title("Distance Error")
    plt.xlabel('Waktu')
    plt.pause(dt)
    

def main():

        '''
        ========================================================================
        POSISI AWAL (INITIAL POSITION) DAN POSISI GOAL
        BESERTA HEADING ATAU ORIENTASINYA
        ========================================================================
        '''
        x_start = 50*random()
        y_start = 50*random()
        
        theta_start = 2 * np.pi * random() - np.pi
        x_goal = 200
        y_goal = 200
        theta_goal = np.pi * random() - np.pi
        print("Initial Position x: %.2f m\nInitial Position y: %.2f m\nInitial theta: %.2f rad\n" %
              (x_start, y_start+10, theta_start))
        print("Goal Position x: %.2f m\nGoal Position y: %.2f m\nGoal theta: %.2f rad\n" %
              (x_goal, y_goal, theta_goal))
        

        '''
        ========================================================================
        JALANKAN BEHAVIOR GO TO GOAL
        ========================================================================
        '''
        move_to_pose(x_start, y_start, theta_start, x_goal, y_goal, theta_goal)
        plt.show()
        

if __name__ == '__main__':
        main()
