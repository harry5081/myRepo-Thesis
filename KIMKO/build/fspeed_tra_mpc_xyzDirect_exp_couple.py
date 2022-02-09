from casadi import *
import numpy as np
import math

def functionTest(v_ref, p_ref, v_init, p_init, v_input_begin, pre_vd_pd, fspeed_ref, fspeed_init):

    #print(v_ref)
    kp=3
    ki=0
    kf=1
    kd=0.3

    kp_z = 2
    ki_z = 0
    kf_z = 1
    kd_z = 0.1


    #a = 0.02
    #az = 0.01
    t = 0.02

    #window = 3
    window =len(p_ref)

    # v_ref=np.array([[0,0,0],[0,0,0],[0,0,0]])
    # p_ref=np.array([[5.504,32.72,0],[21.41,61.84,0],[45.97,84.15,0]])
    # v_init=np.array([0,0,0])
    # p_init=np.array([0,0,0])
    # v_input_begin=np.array([0,0,0])
    # pre_vd_pd=np.array([0,0,0,0,0,0])

    # pre_control = np.array([0,0,0,0,0,0])
    pre_control = pre_vd_pd

    # fspeed_ref=np.array([[100,90],[100,90],[100,90]])



    #############################################################################
    # System Define
    # State
    vx = SX.sym('vx')
    px = SX.sym('px')
    vy = SX.sym('vy')
    py = SX.sym('py')
    vz = SX.sym('vz')
    pz = SX.sym('pz')
    # v =  SX.sym('v',3,1)
    # v[0]=vx
    # v[1]=vy
    # v[2]=vz

    states = vertcat(vx, px, vy, py, vz, pz)

    n_state = states.shape[0]

#     fspeed_vel = sqrt((vx+0.01)**2+(vy+0.01)**2)
#     fspeed_angle = np.arctan2(vx+0.01, vy+0.01) * 180 / np.pi + pz  ### still need unwrap
#     fspeed_states = vertcat(fspeed_vel, fspeed_angle)
#     n_fspeed_states = fspeed_states.shape[0]
#     fs = Function('fs',[vx,vy,pz],[fspeed_states])


    # Control
    vdx = SX.sym('vdx')
    pdx = SX.sym('pdx')
    vdy = SX.sym('vdy')
    pdy = SX.sym('pdy')
    vdz = SX.sym('vdz')
    pdz = SX.sym('pdz')
    controls = vertcat(vdx, pdx, vdy, pdy, vdz, pdz)

    n_controls = controls.shape[0]

    # Mathematical model of the system
    #A = SX([[0,0],[0,1]])
    #Ax_1 = vertcat([1-a,0],[t*cos(pdz*math.pi/360),1])
    #Ax_2 = SX([[0,0],[-t*sin(pdz*math.pi/360),0]])
    Bx = SX([[-kd,-kp]])
    Cx = SX([[kf+kd,kp]])



    #Ay_1 = SX([[0,0],[t*sin(pdz*math.pi/360),0]])
    #Ay_2 = SX([[1-a,0],[t*cos(pdz*math.pi/360),1]])
    By = SX([[-kd,-kp]])
    Cy = SX([[kf+kd,kp]])


    #Az = SX([[1-a,0],[t,1]])
    Bz = SX([[-kd_z,-kp_z]])
    Cz = SX([[kf_z+kd_z,kp_z]])


    zero_22_temp = SX([[0,0],[0,0]])
    zero_02_temp = SX([[0,0]])
    # Ax_temp = SX(horzcat(Ax_1,Ax_2,zero_22_temp))
    # Ay_temp = SX(horzcat(Ay_1,Ay_2,zero_22_temp))
    # Az_temp = SX(horzcat(zero_22_temp,zero_22_temp,Az))

    Bx_temp = SX(horzcat(Bx,zero_02_temp,zero_02_temp))
    By_temp = SX(horzcat(zero_02_temp,By,zero_02_temp))
    Bz_temp = SX(horzcat(zero_02_temp,zero_02_temp,Bz))

    Cx_temp = SX(horzcat(Cx,zero_02_temp,zero_02_temp))
    Cy_temp = SX(horzcat(zero_02_temp,Cy,zero_02_temp))
    Cz_temp = SX(horzcat(zero_02_temp,zero_02_temp,Cz))

    B = SX(vertcat(Bx_temp, By_temp, Bz_temp))
    C = SX(vertcat(Cx_temp, Cy_temp, Cz_temp))

    v_input = B @ states + C @ controls
    v_input_f = Function('v_input_f',[states,controls],[v_input])

    fspeed_vel = sqrt((v_input[0])**2+(v_input[1]+0.01)**2)
    #fs = Function('fs',[states,controls],[fspeed_vel])
    fspeed_angle = np.arctan2(v_input[0], v_input[1]+0.01) * 180 / np.pi  + p_init[2]### still need unwrap
    fspeed_states = vertcat(fspeed_vel, fspeed_angle)
    n_fspeed_states = fspeed_states.shape[0]
    fs = Function('fs',[states,controls],[fspeed_states])


    ex = exp(-3000*t/(fabs(v_input[0]-vx)+200))
    ey = exp(-3000*t/(fabs(v_input[1]-vy)+200))
    ez = exp(-3000*t/(fabs(v_input[2]-vz)+3000))
    #ef = Function('ef',[states,controls],[e])

    A = SX.zeros(6,6)
    A[0,0]=ex
    A[0,1]=0
    A[1,0] = t*cos(pz*2*math.pi/360)
    A[1,1]=1

    A[2,2]=ey
    A[2,3]=0
    A[3,2] = t*cos(pz*2*math.pi/360)
    A[3,3]=1

    A[3,0] = t*sin(pz*2*math.pi/360)
    A[1,2] = -t*sin(pz*2*math.pi/360)

    A[4,4]=ez
    A[4,5]=0
    A[5,4] = t
    A[5,5]=1


    Af = Function('Af',[states,controls],[A])


    h = SX.zeros(6,3)

    h[0,0] = 1-ex
    h[2,1] = 1-ey
    h[4,2] = 1-ez

    hf = Function('hf',[states,controls],[h])

    model = A @ states + h @ B @ states + h @ C @ controls
    f = Function('f',[states,controls],[model])

#     fspeed_vel = sqrt((model[0])**2+(model[2]+0.01)**2)
#     #fs = Function('fs',[states,controls],[fspeed_vel])
#     fspeed_angle = np.arctan2(model[0], model[2]+0.01) * 180 / np.pi + p_init[2]  ### still need unwrap
#     fspeed_states = vertcat(fspeed_vel, fspeed_angle)
#     n_fspeed_states = fspeed_states.shape[0]
#     fs = Function('fs',[states,controls],[fspeed_states])



    ###########################################################################
    # Make mathematical model as function object



    U = SX.sym('U',n_controls,window) # vd,pd during a window
    P = SX.sym('P',n_state + window*n_state + window*n_fspeed_states) # initial state and reference state of the robot
    X = SX.sym('X',n_state,(window+1)) # states during prediction horizontal

    V_INPUT_MATRIX = SX.sym('V_INPUT_MATRIX',3,window)
    FSPEED_STATE = SX.sym('FSPEED_STATE',2,window)

    # Construct obj function
    obj = 0
    g=[]

    Q = np.zeros((6,6))
    Q[0,0]=0
    Q[1,1]=3
    Q[2,2]=0
    Q[3,3]=3
    Q[4,4]=0
    Q[5,5]=3

    R = np.zeros((3,3))
    R[0,0]=1
    R[1,1]=1
    R[2,2]=0.5

    R1 = np.zeros((3,3))
    R1[0,0]=1
    R1[1,1]=1
    R1[2,2]=0.5

    R2 = np.zeros((6,6))
    R2[0,0]=0
    R2[1,1]=0
    R2[2,2]=0
    R2[3,3]=0
    R2[4,4]=0
    R2[5,5]=0

    R_fs = np.zeros((2,2))
    R_fs[0,0]=20
    R_fs[1,1]=5

    v_input_temp = v_input_begin

    ############################################################################    
    # Calculate in a prediction horizontal

    #X[:,0]=P[0:2] # assign the initial state to the robot
    state_current = X[:,0]
    g = vertcat(g,state_current-P[0:6]) 

    for i in range(window): # fill out all prediction state within a predic horz

        state_current = X[:,i]
        control_current = U[:,i]

        ref_current = P[(i+1)*n_state:(i+2)*n_state]
        fspeed_ref_current = P[(1+window)*n_state + i*n_fspeed_states:(1+window)*n_state +i*n_fspeed_states+n_fspeed_states]
        

        control_diff = control_current-pre_control

        V_INPUT_MATRIX[:,i]= v_input_f(state_current,control_current)
        v_input_dff = V_INPUT_MATRIX[:,i]- v_input_temp
        v_input_temp = V_INPUT_MATRIX[:,i]

        FSPEED_STATE[:,i]= fs(state_current,control_current)
        fspeed_err=FSPEED_STATE[:,i]-fspeed_ref_current

      #  obj = ( obj + 10*(X[:,i+1] - ref_current).T @ Q @ (X[:,i+1] - ref_current) +
      #          fspeed_err.T @ R_fs @ fspeed_err+
      #          +1*(V_INPUT_MATRIX[:,i].T @ V_INPUT_MATRIX[:,i]+ 3*v_input_dff.T @ v_input_dff) )# + (control_diff.T @ R2 @control_diff)
        obj = obj + (fspeed_err.T @ R_fs @ fspeed_err) + 10*(X[:,i+1] - ref_current).T @ Q @ (X[:,i+1] - ref_current) + 1*(V_INPUT_MATRIX[:,i].T @ R @ V_INPUT_MATRIX[:,i]+ 3*v_input_dff.T @ R1 @ v_input_dff)
        
        #obj = obj + 10*(FSPEED_STATE[:,i][0]-fspeed_ref_current[0])**2+ 0.1*(FSPEED_STATE[:,i][1]-fspeed_ref_current[1])**2+1*(V_INPUT_MATRIX[:,i].T @ V_INPUT_MATRIX[:,i]+ 3*v_input_dff.T @ v_input_dff)
        #+ 0.1*(FSPEED_STATE[:,i][1]-90)**2 +1*(V_INPUT_MATRIX[:,i].T @ V_INPUT_MATRIX[:,i]+ 3*v_input_dff.T @ v_input_dff)
        #1*(V_INPUT_MATRIX[:,i].T @ V_INPUT_MATRIX[:,i]+ 3*v_input_dff.T @ v_input_dff)
        # + 0.0001*(FSPEED_STATE[:,i][1]-fspeed_ref_current[1])**2+ 1*(V_INPUT_MATRIX[:,i].T @ V_INPUT_MATRIX[:,i])
        
        # + 0.0001*(FSPEED_STATE[:,i][1]-fspeed_ref_current[1])**2
        
        
        state_next_multi_shoot = X[:,i+1]

        state_next = f(state_current,control_current)
        g = vertcat(g,state_next_multi_shoot-state_next)
        #X[:,i+1] = state_next
        
    v_input_ff = Function('v_input_ff',[U,X],[V_INPUT_MATRIX])
        
    ###############################################################################
    # Construct NLP solver
    STATE_variables = reshape(X,6*(window+1),1) #[X0, X1, X2, X3...]
    INPUT_variables = reshape(U,6*window,1) #[vd1, pd1, vd2, pd2, vd3, pd3 ...]
    OPT_variables = vertcat(INPUT_variables,STATE_variables)
    #print(OPT_variables)

    g = vertcat(g,reshape(V_INPUT_MATRIX,3*window,1))

    nlp_prob = {'f':obj, 'x':OPT_variables, 'g':g, 'p':P}   #dict


    # Options For Solver
    #opts = {"print_time": False, "ipopt.max_iter": 10}
    opts = {}
    opts["print_time"] = False

    ipopt_options={}
    ipopt_options["max_iter"] = 100#100
    ipopt_options["print_level"] = 0
    ipopt_options["acceptable_tol"] = 1e-6#1e-6#
    ipopt_options["acceptable_obj_change_tol"] = 1e-6#1e-6

    opts["ipopt"]=ipopt_options


    ###############################################################################
    # Execute solver
    args = {}
    args["lbx"] = -float('inf')
    args["ubx"] = float('inf')
    #args["lbg"] = -float('inf')

    #args["ubg"] = 250*np.ones(window)
    #args["lbg"] = -float('inf')
    #args["ubg"] = float('inf')

    v_input_g_temp = np.array([250,250,15])
    args["lbg"] = vertcat(np.zeros(6*(window+1)),-1*repmat(v_input_g_temp,(window),1))
    args["ubg"] = vertcat(np.zeros(6*(window+1)),repmat(v_input_g_temp,(window),1))


    solver = nlpsol('solver', 'ipopt', nlp_prob, opts)

    ########################################################################################################
    init_state_temp = np.array([v_init,p_init])
    init_state = reshape(init_state_temp,6,1)
    ########################################################################################################
    temp = int(window*n_state/2)
    ref_state_temp = np.concatenate((np.reshape(v_ref,(-1,temp)),np.reshape(p_ref,(-1,temp))),axis=0)
    ref_state = reshape(ref_state_temp,window*n_state,1)
    #########################################################################################################
    #fspeed_ref = np.reshape(fspeed_ref,n_fspeed_states*window,1)
    
    fspeed_ref=np.reshape(fspeed_ref,n_fspeed_states*window)
    #fspeed_ref=np.reshape(fspeed_ref,(n_fspeed_states*window,1))
    #print(fspeed_ref[0],fspeed_ref[2],fspeed_ref[4],fspeed_ref[6])
    #fspeed_ref = np.array((fspeed_ref))
    


    #u0 = np.zeros((6,window)) # u0 used for initial guess of [vd1, pd1, vd2, pd2, vd3, pd3 ...]
    #u0 = repmat(pre_control,window,1)
    #u0 = repmat(final_state,window,1)
    u0 = repmat(init_state,window,1)

    #init_v_input = np.array([0])
    #args["p"] = np.concatenate((init_state, final_state), axis=None)

      

    args["p"] =vertcat(init_state, ref_state,fspeed_ref)
    args["x0"] = vertcat(reshape(u0,6*window,1),repmat(init_state,(window+1),1)) #initial guess of [vd1, pd1, vd2, pd2, vd3, pd3 ...]
    


    sol = solver(lbx=args["lbx"], ubx=args["ubx"], lbg=args["lbg"], ubg=args["ubg"],\
            p=args["p"], x0=args["x0"])



    #CAN_WRITE to make robot move
    mpc_iter = 0
    x_sol = sol['x']

    #print(x_sol)

    u_sol =x_sol[0:6*(window)]
    state_sol=x_sol[6*(window):6*(window+1)+6*window]

    # #print(u_sol)
    # #print(state_sol)

    # u = reshape(u_sol,6,window) 
    # state = reshape(state_sol,6,window+1) 

    # # record the solved input vd, pd
    # u_predichorz_update = np.zeros((1,6,window))
    # u_predichorz_update[0,:,:] = u

    # # calculate all the state within a predic_horz
    # states_predichorz_update = np.zeros((1,6,window+1))
    # states_predichorz_update[0,:,:] = state # get all states within a predic horz
    # #print(states_predichorz_update)
    # #print(u_predichorz_update)

    # # calculate all the v_input within a predic_horz
    # VVV = v_input_ff(u,state)
    # v_predichorz_update = np.zeros((1,3,window))
    # v_predichorz_update[0,:,:] = VVV

    # # np.set_printoptions(precision=2,suppress=True)
    # # for i in range(1):
    # #     for j in range(window):
    # #         print("----Window Update----")
    # #         print("window: ", j)
    # #         print("state [v, p] = ",states_predichorz_update[i,:,j])
    # #         print("input [vd, pd] = ",u_predichorz_update[i,:,j])
    # #         print("v_input = ",v_predichorz_update[i,:,j])
    # #         print("")
    # #         print("")


    return list([u_sol[0],u_sol[1],u_sol[2],u_sol[3],u_sol[4],u_sol[5]])
        # print(x_sol[0:2])

        #return list([x_sol[0],x_sol[1]])






# ##############################################################################
# #Update the prediction horizontal
# i=0
# diff = args["p"][0:6]-final_state
# #while (abs(args["p"][5]-final_state[5]) > 1e-1):
# #while (abs(args["p"][1]-final_state[1])+abs(args["p"][3]-final_state[3])+abs(args["p"][5]-final_state[5]) > 10*(1e-1)):
# while (diff.T @ diff > 10): 
#     diff = args["p"][0:6]-final_state
#     i=i+1
# #for i in range(100):
#     #print("----- Horizontal Prediction Update -----")
#     #shift
#     init_state = states_predichorz_update[mpc_iter,:,1]
#     # final_state = np.array([[0],[p_ref]])
    
#     # init_state_temp = np.array([v_init,p_init])
#     # init_state = reshape(init_state_temp,6,1)

#     # final_state_temp = np.array([v_ref,p_ref])
#     # final_state = reshape(final_state_temp,6,1)
    
#     state_0 = state_sol
#     u0 = u_sol #u0 used for initial guess of [vd1, pd1, vd2, pd2, vd3, pd3 ...]
#     mpc_iter = mpc_iter+1
#     #print("MPC Iteration: ", mpc_iter)
    
#     args["p"] =vertcat(init_state, final_state)
#     #args["x0"] = reshape(u0,2*window,1) #initial guess of [vd1, pd1, vd2, pd2, vd3, pd3 ...]
#     args["x0"] =vertcat(reshape(u0,6*window,1),repmat(init_state,(window+1),1)) #initial guess of [vd1, pd1, vd2, pd2, vd3, pd3 ...]
#     #args["x0"] = vertcat(reshape(u0,2*window,1),reshape(state_0,2*(window+1),1)) #initial guess of [vd1, pd1, vd2, pd2, vd3, pd3 ...]

#     sol = solver(lbx=args["lbx"], ubx=args["ubx"], lbg=args["lbg"], ubg=args["ubg"],\
#             p=args["p"], x0=args["x0"])
    
    
#     #CAN_WRITE to make robot move
#     x_sol = sol['x']

#     u_sol =x_sol[0:6*(window)]
#     state_sol=x_sol[6*(window):6*(window+1)+6*window]
    
#     u = reshape(u_sol,6,window) 
#     state = reshape(state_sol,6,window+1) 
    
    
#     # record the solved input vd, pd
#     u_predichorz_update= np.concatenate([u_predichorz_update,np.reshape(u,(1,6,window))],0)
        
#     # calculate all the state within a predic_horz
#     states_through_predihorz = state # get all states within a predic horz
#     # when update a horizontal, add a new page in states_predichorz_update
#     states_predichorz_update = np.concatenate([states_predichorz_update,np.reshape(states_through_predihorz,(1,6,window+1))],0)
    
#     VVV = v_input_ff(u,state)
#     v_predichorz_update = np.concatenate([v_predichorz_update,np.reshape(VVV,(1,3,window))],0)



#     # print(states_predichorz_update)
#     # print(u_predichorz_update)

#     # np.set_printoptions(precision=2,suppress=True)
#     # for i in range(1):
#     #     for j in range(window):
#     #         print("----Window Update----")
#     #         print("window: ", j)
#     #         print("state [v, p] = ",states_predichorz_update[i,:,j])
#     #         print("input [vd, pd] = ",u_predichorz_update[i,:,j])
#     #         print("v_input = ",VVV[j])
#     #         print("")
#     #         print("")

# np.set_printoptions(precision=2,suppress=True)
# for i in range(mpc_iter+1):
#     print("----Prediction Horizontal Update----")
#     print("mpc_iter: ", i)
#     print("state [v, p] = ",states_predichorz_update[i,:,0])
#     print("demand [vd, pd] = ",u_predichorz_update[i,:,0])
#     print("v_input = ",v_predichorz_update[i,:,0])
#     print("")
#     print("")

    
            