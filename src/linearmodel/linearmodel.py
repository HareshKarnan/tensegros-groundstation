#!/usr/bin/env python
import rospy
from std_msgs.msg import Float32MultiArray
from sympy import *
from sympy.physics.mechanics import *
from numpy.linalg import *
from numpy import *


def callback(data,args):
    th1, th2, th3 = dynamicsymbols('th1 th2 th3')  # generalized coordinates theta
    phi1, phi2, phi3 = dynamicsymbols('phi1 phi2 phi3')  # generalized coordinated phi

    th1d, th2d, th3d = dynamicsymbols('th1 th2 th3', 1)
    phi1d, phi2d, phi3d = dynamicsymbols('phi1 phi2 phi3', 1)
    t1, t2, t3, t4, t5, t6 = dynamicsymbols('t1 t2 t3 t4 t5 t6')
    length_bar_com, m, g, t = symbols('length_bar_com m g t')  # comlength mass gravity
    edge_length = symbols('edge_length')

    LM = args
    tn1 = data.data[0]
    tn2 = data.data[1]
    tn3 = data.data[2]
    tn4 = data.data[3]
    tn5 = data.data[4]
    tn6 = data.data[5]
    thv1 = data.data[6]
    phv1 = data.data[7]
    thv2 = data.data[8]
    phv2 = data.data[9]
    thv3 = data.data[10]
    phv3 = data.data[11]
    oppt = {t1: tn1, t2: tn2, t3: tn3, t4: tn4, t5: tn5, t6: tn6, th1: thv1, th2: thv2, th3: thv3,
            phi1: phv1, phi2: phv2, phi3: phv3, th1d: 0, th2d: 0, th3d: 0, phi1d: 0, phi2d: 0,
            phi3d: 0, th1d.diff(t): 0, th2d.diff(t): 0, th3d.diff(t): 0}
    A, B, inp_vec = LM.linearize(q_ind=[th1, th2, th3, phi1, phi2, phi3],
                                 qd_ind=[th1d, th2d, th3d, phi1d, phi2d, phi3d], A_and_B=True,op_point=oppt)
    paramset = {m: 0.06803886, length_bar_com: 5, edge_length: 7.05}
    B = msubs(B,paramset)
    A = msubs(A,paramset)
    pprint(B)
    # # A = A.tolist()
    # pub = rospy.Publisher('linearized', Float32MultiArray, queue_size=72)
    # B = B.tolist()
    # pub.publish(Float32MultiArray(data=B))


def listener(LM):
    rospy.init_node('linearmodel',anonymous = False)
    rospy.Subscriber('tensionvals',Float32MultiArray,callback,LM)
    # keep python from exiting

    rospy.spin()

def dynamicsrun():
    nb = 3  # number of bars

    th1, th2, th3 = dynamicsymbols('th1 th2 th3')  # generalized coordinates theta
    phi1, phi2, phi3 = dynamicsymbols('phi1 phi2 phi3')  # generalized coordinated phi

    th1d, th2d, th3d = dynamicsymbols('th1 th2 th3', 1)
    phi1d, phi2d, phi3d = dynamicsymbols('phi1 phi2 phi3', 1)

    I = ReferenceFrame('I')  # inertial reference frame
    O = Point('O')  # centroid of bottom triangle
    O.set_vel(I, 0)  # point doesnt move.

    edge_length = symbols('edge_length')

    # now define the bottom 3 joints and frames
    # N1 = I.orientnew('N1','Axis',[0,I.z])
    # N1.set_ang_vel(I,0)
    n1 = Point('n1')
    n1.set_pos(O, -edge_length / 2.0 * I.x - edge_length / (2.0 * sqrt(3.0)) * I.y)
    n1.set_vel(I, 0)

    # N2 = I.orientnew('N2','Axis',[120.0*(pi/180.0),I.z])
    # N2.set_ang_vel(I,0)
    n2 = Point('n2')
    n2.set_pos(n1, edge_length * I.x)
    n2.set_vel(I, 0)

    # N3 = I.orientnew('N3','Axis',[210.0*(pi/180.0),I.z])
    # N3.set_ang_vel(I,0)
    n3 = Point('n3')
    n3.set_pos(O, edge_length / sqrt(3.0) * I.y)
    n3.set_vel(I, 0)

    length_bar_com, m, g, t = symbols('length_bar_com m g t')  # comlength mass gravity

    # initialize bar frame
    B1 = I.orientnew('B1', 'Body', [th1, phi1, 0], 'ZYX')
    B2 = I.orientnew('B2', 'Body', [th2 + 120.0 * (pi / 180.0), phi2, 0], 'ZYX')
    B3 = I.orientnew('B3', 'Body', [th3 + 210.0 * (pi / 180.0), phi3, 0], 'ZYX')
    B1.set_ang_vel(I, th1d * I.z + phi1d * B1.y)
    B2.set_ang_vel(I, th2d * I.z + phi2d * B2.y)
    B3.set_ang_vel(I, th3d * I.z + phi3d * B3.y)

    com1 = Point('com1')
    com1.set_pos(n1, length_bar_com * B1.x)
    com1.v2pt_theory(n1, I, B1)
    com2 = Point('com2')
    com2.set_pos(n2, length_bar_com * B2.x)
    com2.v2pt_theory(n2, I, B2)
    com3 = Point('com3')
    com3.set_pos(n3, length_bar_com * B3.x)
    com3.v2pt_theory(n3, I, B3)

    # inertia tuples
    b1_inertia = inertia(B1, 0, (1 / 3.0) * m * (length_bar_com * 2) ** 2, (1 / 3.0) * m * (length_bar_com * 2) ** 2)
    b1_inertia.to_matrix(B1)
    b1_inertia = (b1_inertia, com1)  # create the tuple

    b2_inertia = inertia(B2, 0, (1 / 3.0) * m * (length_bar_com * 2) ** 2, (1 / 3.0) * m * (length_bar_com * 2) ** 2)
    b2_inertia.to_matrix(B2)
    b2_inertia = (b2_inertia, com2)  # create the tuple

    b3_inertia = inertia(B3, 0, (1 / 3.0) * m * (length_bar_com * 2) ** 2, (1 / 3.0) * m * (length_bar_com * 2) ** 2)
    b3_inertia.to_matrix(B3)
    b3_inertia = (b3_inertia, com3)  # create the tuple

    # define rigid body
    bar1 = RigidBody('bar1', com1, B1, m, b1_inertia)
    bar2 = RigidBody('bar2', com2, B2, m, b2_inertia)
    bar3 = RigidBody('bar3', com3, B3, m, b3_inertia)

    n4 = Point('n4')
    n5 = Point('n5')
    n6 = Point('n6')
    n4.set_pos(n3, length_bar_com * 2 * B3.x)
    n4.v2pt_theory(n3, I, B3)
    n5.set_pos(n1, length_bar_com * 2 * B1.x)
    n5.v2pt_theory(n1, I, B1)
    n6.set_pos(n2, length_bar_com * 2 * B2.x)
    n6.v2pt_theory(n2, I, B2)
    t1, t2, t3, t4, t5, t6 = dynamicsymbols('t1 t2 t3 t4 t5 t6')

    # f1 = t1*n6.pos_from(n2).normalize()
    # f2 = t2*n4.pos_from(n3).normalize()
    # f3 = t3*n5.pos_from(n1).normalize()
    # f4 = t4*n6.pos_from(n5).normalize()
    # f5 = t5*n4.pos_from(n6).normalize()
    # f6 = t6*n5.pos_from(n4).normalize()
    f1 = t1 * n6.pos_from(n3).normalize()
    f2 = t2 * n4.pos_from(n1).normalize()
    f3 = t3 * n5.pos_from(n2).normalize()
    f4 = t4 * n6.pos_from(n5).normalize()
    f5 = t5 * n4.pos_from(n6).normalize()
    f6 = t6 * n5.pos_from(n4).normalize()

    # fw = -(m/2.0)*g*I.z
    forces = [(n1, f2), (n2, f3), (n3, f1), (n4, f6 - f5 - f2), (n5, f4 - f6 - f3), (n6, -f1 - f4 + f5)]
    L = Lagrangian(I, bar1, bar2, bar3)
    LM = LagrangesMethod(L, qs=[th1, th2, th3, phi1, phi2, phi3], forcelist=forces, frame=I)
    eq = LM.form_lagranges_equations()
    eq = LM.mass_matrix_full.inv() * LM.forcing_full
    paramset = {m: 0.06803886, length_bar_com: 5, edge_length: 7.05}
    eq = msubs(eq, paramset)
    return LM

if __name__ == '__main__':
    LM=dynamicsrun()
    try:
        listener(LM)
    except rospy.ROSInterruptException:
        rospy.loginfo("Linear Model error")
