#! /usr/bin/python3
import roboticstoolbox as rtb
import spatialmath as sm
from sympy import *
import numpy as np
import math
import matplotlib.pyplot as plt
import open3d as o3d

def DHRobot(d1 = 0.18061, d2 = 0.00665, a1= -0.00654 , a2=0.0, alpha1=math.pi/2, alpha2=0,offset1=-math.pi/2,offset2=0):
    Link1 = rtb.RevoluteDH(alpha=alpha1, a=a1, d=d1,offset=offset1,qlim=[-math.pi/2,math.pi/2])
    Link2 = rtb.RevoluteDH(alpha=alpha2, a=a2, d=d2,offset=offset2,qlim=[-math.pi/2,math.pi/2])
    Link3 = rtb.RevoluteDH(a=0.05,qlim=[0,0])
    name="gimbal_robot"

    base=sm.SE3(0.0, 0.0, 0.0)*sm.SE3.Rx(0)*sm.SE3.Ry(0)*sm.SE3.Rz(0)

    robot = rtb.SerialLink([Link1, Link2,Link3], name=name, base=base)
    #add joints limits

    robot.q=[0.0,0.0,0.0]
    return robot


def robotNormals(robot,robot_base,target_point=[0.0,0.0,0.0]):
    robot.base=robot_base
    work_space = []
    for i in np.linspace(-math.pi/2.5, math.pi/2.5, 50):
            for j in np.linspace(-math.pi/2.5, math.pi/2.5, 50):
                work_space.append(robot.fkine([i, j,0]).t)

    work_space = np.array(work_space)
    # print(work_space)
    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(work_space)
    pcd.estimate_normals()
    pcd.orient_normals_towards_camera_location(robot_base.t)
    o3d.visualization.draw_geometries([pcd],point_show_normal=True)
    normal_vectors=np.asarray(pcd.normals)
    
    #calculate the distance between the line and the point
    dist=np.zeros((len(work_space),1))
    for i in range(len(work_space)):
        dist[i]=np.linalg.norm(np.cross(work_space[i]-np.array(target_point),normal_vectors[i]))/np.linalg.norm(normal_vectors[i])
    
    
    return np.array(work_space[np.argmin(dist),:]), normal_vectors[np.argmin(dist),:]


def inverseKinematics(robot,work_space_target):
    T = sm.SE3(work_space_target[0],work_space_target[1],work_space_target[2])
    T.print()
    Ttrans=sm.SE3(0,0,0)

    #na simulacao o metodo utilizando as trasnsfromacoes de rotacao de pitch e yaw nao funcionou de maneira satisfatoria
    sol_trasnf = robot.ikine_LM(Ttrans,mask=[0,0,0,1,1,1],search=False)
    print(robot.base)
    sol_norm = robot.ikine_LM(T,mask=[1,1,1,0,0,0],search=False)

    return sol_norm[0]


# TODO: FIX VISUALIZATION
# def visualize(robot,work_space,normal_vectors,target_point):
# # print(sol_trasnf)
# # print(sol_norm)
# # robot.plot(sol_trasnf[0],block=True)
# # robot.plot(sol_norm[0],block=True)
# # robot.plot(sol_trasnf)
#     # robot.plot(sol_norm)
#     fig = plt.figure()
#     ax = fig.add_subplot(projection='3d')
#         #3d plot of forward kinematics
#     ax.axes.set_xlim3d(left=0, right=0.5) 
#     ax.axes.set_ylim3d(bottom=0, top=0.5) 
#     ax.axes.set_zlim3d(bottom=-0.5, top=0)

#     ax.scatter(work_space[:,0], work_space[:,1], work_space[:,2])
#     ax.scatter(np.array(target_point), c='b', marker='o')

#     ax.quiver(work_space[np.argmin(dist),0], work_space[np.argmin(dist),1], work_space[np.argmin(dist),2], normal_vectors[np.argmin(dist),0], normal_vectors[np.argmin(dist),1], normal_vectors[np.argmin(dist),2],length=1)

#     plt.show()


def inverse_kinematics(robot=DHRobot(),robot_base=sm.SE3(1.0, 0.0, .5)):
    work_space_target, normal_vectors = robotNormals(robot, robot_base)
    print(work_space_target)
    q=inverseKinematics(robot,work_space_target)
    print('inverse kinematics:',q)
    return (q[0],q[1])
    # visualize(robot,work_space,normal_vectors,[0.0,0.0,0.0])
if __name__ == "__main__":
    inverse_kinematics()