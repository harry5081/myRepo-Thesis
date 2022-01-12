from casadi import *
import numpy as np
def functionTest(v_ref, p_ref, v_init, p_init, v_input_begin, pre_vd_pd):

    kp=3
    ki=0
    kf=1
    kd=0.3
    
    kp_z = 3
    ki_z = 0
    kf_z = 1
    kd_z = 1
    

    a = 0.02
    t = 0.02

    window = 20

    # v_ref=np.array([0,0,0])
    # p_ref=np.array([100,0,0])
    # v_init=np.array([0,0,0])
    # p_init=np.array([0,0,0])
    # v_input_begin=np.array([0,0,0])

    pre_control = pre_vd_pd



    #############################################################################
    # System Define
    # State
    vx = SX.sym('vx')
    px = SX.sym('px')
    vy = SX.sym('vy')
    py = SX.sym('py')
    vz = SX.sym('vz')
    pz = SX.sym('pz')
    states = vertcat(vx, px, vy, py, vz, pz)

    n_state = states.shape[0]

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
    
    Bx = SX([[-kd,-kp]])
    Cx = SX([[kf+kd,kp]])

    By = SX([[-kd,-kp]])
    Cy = SX([[kf+kd,kp]])

    Bz = SX([[-kd_z,-kp_z]])
    Cz = SX([[kf_z+kd_z,kp_z]])

    zero_22_temp = SX([[0,0],[0,0]])
    zero_02_temp = SX([[0,0]])

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

    e = exp(-200*t/(fabs(v_input-v)+0.01))
    ef = Function('ef',[states,controls],[e])

    #######################################################################################
    
    #A = SX([[0,0],[0,1]])
    Ax = SX([[1-a,0],[t,1]])
    
   
    #h = SX([[a],[t]])
    hx= SX([[a],[0]])


    Ay = SX([[1-a,0],[t,1]])
    
    #h = SX([[a],[t]])
    hy = SX([[a],[0]])

    Az = SX([[1-a,0],[t,1]])
    
    #h = SX([[a],[t]])
    hz = SX([[a],[0]])

    
    Ax_temp = SX(horzcat(Ax,zero_22_temp,zero_22_temp))
    Ay_temp = SX(horzcat(zero_22_temp,Ay,zero_22_temp))
    Az_temp = SX(horzcat(zero_22_temp,zero_22_temp,Az))

    

    hx_temp = SX(horzcat(hx,zero_02_temp.T,zero_02_temp.T))
    hy_temp = SX(horzcat(zero_02_temp.T,hy,zero_02_temp.T))
    hz_temp = SX(horzcat(zero_02_temp.T,zero_02_temp.T,hz))


    A = SX(vertcat(Ax_temp, Ay_temp, Az_temp))
    
    h = SX(vertcat(hx_temp, hy_temp, hz_temp))


    model = A @ states + h @ B @ states + h @ C @ controls
    v_input = B @ states + C @ controls

    ###########################################################################
    # Make mathematical model as function object
    f = Function('f',[states,controls],[model])
    v_input_f = Function('v_input_f',[states,controls],[v_input])

    U = SX.sym('U',n_controls,window) # vd,pd during a window
    P = SX.sym('P',n_state + n_state) # initial state and reference state of the robot
    X = SX.sym('X',n_state,(window+1)) # states during prediction horizontal

    V_INPUT_MATRIX= SX.sym('V_INPUT_MATRIX',3,window)

    # Construct obj function
    obj = 0
    g=[]

    Q = np.zeros((6,6))
    Q[0,0]=0
    Q[1,1]=1
    Q[2,2]=0
    Q[3,3]=1
    Q[4,4]=0
    Q[5,5]=1

    #print(Q)

    R = np.zeros((6,6))
    R[0,0]=1
    R[1,1]=1
    #print(R)

    v_input_temp = v_input_begin

    ############################################################################    
    # Calculate in a prediction horizontal

    #X[:,0]=P[0:2] # assign the initial state to the robot
    state_current = X[:,0]
    g = vertcat(g,state_current-P[0:6]) 



    for i in range(window): # fill out all prediction state within a predic horz

        state_current = X[:,i]
        control_current = U[:,i]

        control_diff = control_current-pre_control

        V_INPUT_MATRIX[:,i]= v_input_f(state_current,control_current)
        #v_input_dff = V_INPUT_MATRIX[:,i]- v_input_temp.T
        #v_input_temp = V_INPUT_MATRIX[:,i]

        #obj = obj + 1.5*i*(X[:,i+1] - P[6:12]).T @ Q @ (X[:,i+1] - P[6:12]) + V_INPUT_MATRIX[:,i].T @ V_INPUT_MATRIX[:,i]
        obj = obj + 10*(X[:,i+1] - P[6:12]).T @ Q @ (X[:,i+1] - P[6:12])+ V_INPUT_MATRIX[:,i].T @ V_INPUT_MATRIX[:,i]+ 0.001*(control_diff.T @ control_diff)


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

    v_input_g_temp = np.array([250,250,5])
    args["lbg"] = vertcat(np.zeros(6*(window+1)),-1*repmat(v_input_g_temp,(window),1))
    args["ubg"] = vertcat(np.zeros(6*(window+1)),repmat(v_input_g_temp,(window),1))


    solver = nlpsol('solver', 'ipopt', nlp_prob, opts)


    init_state_temp = np.array([v_init,p_init])
    init_state = reshape(init_state_temp,6,1)

    final_state_temp = np.array([v_ref,p_ref])
    final_state = reshape(final_state_temp,6,1)

    u0 = np.zeros((6,window)) # u0 used for initial guess of [vd1, pd1, vd2, pd2, vd3, pd3 ...]


    #init_v_input = np.array([0])
    #args["p"] = np.concatenate((init_state, final_state), axis=None)

    args["p"] =vertcat(init_state, final_state)
    args["x0"] = vertcat(reshape(u0,6*window,1),repmat(init_state,(window+1),1)) #initial guess of [vd1, pd1, vd2, pd2, vd3, pd3 ...]



    sol = solver(lbx=args["lbx"], ubx=args["ubx"], lbg=args["lbg"], ubg=args["ubg"],\
            p=args["p"], x0=args["x0"])



    #CAN_WRITE to make robot move
    mpc_iter = 0
    x_sol = sol['x']

    #print(x_sol)

    u_sol =x_sol[0:6*(window)]
    state_sol=x_sol[6*(window):6*(window+1)+6*window]

    #print(u_sol)
    #print(state_sol)

    u = reshape(u_sol,6,window) 
    state = reshape(state_sol,6,window+1) 

    # record the solved input vd, pd
    u_predichorz_update = np.zeros((1,6,window))
    u_predichorz_update[0,:,:] = u

    # calculate all the state within a predic_horz
    states_predichorz_update = np.zeros((1,6,window+1))
    states_predichorz_update[0,:,:] = state # get all states within a predic horz
    #print(states_predichorz_update)
    #print(u_predichorz_update)

    # calculate all the v_input within a predic_horz
    VVV = v_input_ff(u,state)
    v_predichorz_update = np.zeros((1,3,window))
    v_predichorz_update[0,:,:] = VVV

    # np.set_printoptions(precision=2,suppress=True)
    # for i in range(1):
    #     for j in range(window):
    #         print("----Window Update----")
    #         print("window: ", j)
    #         print("state [v, p] = ",states_predichorz_update[i,:,j])
    #         print("input [vd, pd] = ",u_predichorz_update[i,:,j])
    #         print("v_input = ",v_predichorz_update[i,:,j])
    #         print("")
    #         print("")

    
    return list([u_sol[0],u_sol[1],u_sol[2],u_sol[3],u_sol[4],u_sol[5]])

    
def functionTest(v_ref=0, p_ref=100, v_init=0, p_init=0, v_input_begin=0):
#def functionTest():

    kf=1
    kd=0.3
    kp=3
    ki=0

    a = 0.02
    t = 0.02

    window = 20

    v_ref=0
    p_ref=100
    v_init=0
    p_init=0
    v_input_begin=0

    #############################################################################
    # System Define
    # State
    v = SX.sym('v')
    p = SX.sym('p')
    states = vertcat(v,p)

    n_state = states.shape[0]

    # Control
    v_d = SX.sym('v_d')
    p_d = SX.sym('p_d')
    controls = vertcat(v_d, p_d)

    n_controls = controls.shape[0]

    # Mathematical model of the system
    B = SX([[-kd,-kp]])
    C = SX([[kf+kd,kp]])

    v_input = B @ states + C @ controls
    v_input_f = Function('v_input_f',[states,controls],[v_input])

    e = exp(-200*t/(fabs(v_input-v)+0.01))
    ef = Function('ef',[states,controls],[e])

    #A = SX([[0,0],[0,1]])
    #A = SX([[1-a,0],[t,1]])
    #A = SX([[e,0],[t,1]])
    # A_temp1=horzcat(e,0)
    # A_temp2=horzcat(t,1)
    # A = vertcat(A_temp1,A_temp2)
    #A = SX([[e,0],[t,1]])
    A = SX.sym('A',2,2)
    A[0,0]=e
    A[0,1]=0
    A[1,0]=t
    A[1,1]=1
    Af = Function('Af',[states,controls],[A])

    #h = SX([[a],[t]])
    #h = SX([[a],[0]])
    #h = SX([[1-e],[0]])
    #h = vertcat(1-e,0)
    h = SX.sym('h',2,1)
    h[0,0] = 1-e
    h[1,0] = 0
    hf = Function('hf',[states,controls],[h])

    model = A @ states + h @ B @ states + h @ C @ controls

    ###########################################################################
    # Make mathematical model as function object
    f = Function('f',[states,controls],[model])


    U = SX.sym('U',n_controls,window) # vd,pd during a window
    P = SX.sym('P',n_state + n_state) # initial state and reference state of the robot
    X = SX.sym('X',n_state,(window+1)) # states during prediction horizontal

    V_INPUT_MATRIX= SX.sym('V_INPUT_MATRIX',window)

    ############################################################################    
    # Calculate in a prediction horizontal
    X[:,0]=P[0:2] # assign the initial state to the robot
    for i in range(window): # fill out all prediction state within a predic horz

        state_current = X[:,i]
        control_current = U[:,i]
        V_INPUT_MATRIX[i]= v_input_f(state_current,control_current)

        #e_factor = ef(state_current,control_current)
        #A_factor = Af(state_current,control_current)
        #h_factor = hf(state_current,control_current) 
        #state_next = A_factor @ state_current + h_factor @ B @ state_current + h_factor @ C @ control_current
        
        #state_next = f(state_current,control_current)

        e = exp(-200*t/(fabs(V_INPUT_MATRIX[i]-state_current[0])+0.01))
        A = SX.sym('A',2,2)
        A[0,0]=e
        A[0,1]=0
        A[1,0]=t
        A[1,1]=1
        
        h = SX.sym('h',2,1)
        h[0,0] = 1-e
        h[1,0] = 0

        state_next = A @ state_current + h @ B @ state_current + h @ C @ control_current
        

        X[:,i+1] = state_next
        
        

    ff = Function('ff',[U,P],[X]) # to get all data throughout predict horz
    v_input_ff = Function('v_input_ff',[U,P],[V_INPUT_MATRIX])
    ###############################################################################
    # Construct obj function
    obj = 0

    Q = np.zeros((2,2))
    Q[0,0]=0
    Q[1,1]=1
    #print(Q)

    R = np.zeros((2,2))
    R[0,0]=1
    R[1,1]=1
    #print(R)

    v_input_temp = v_input_begin

    for i in range(0, window):
        
        state = X[:,i+1]
        control = U[:,i]
        
        v_input_dff = V_INPUT_MATRIX[i]- v_input_temp
        v_input_temp = V_INPUT_MATRIX[i]
        #obj = obj + i*(state - P[2:4]).T @ Q @ (state - P[2:4]) + 3*V_INPUT_MATRIX[i] **2
        obj = obj + 10*(state - P[2:4]).T @ Q @ (state - P[2:4])+ V_INPUT_MATRIX[i] **2# + control.T @ R @ control
        
    ###############################################################################
    # Construct NLP solver
    OPT_variables = reshape(U,2*window,1) #[vd1, pd1, vd2, pd2, vd3, pd3 ...]
    g = V_INPUT_MATRIX
    #g=[]

    nlp_prob = {'f':obj, 'x':OPT_variables, 'g':g, 'p':P}   #dict

    # Options For Solver
    #opts = {"print_time": False, "ipopt.max_iter": 10}
    opts = {}
    opts["print_time"] = False

    ipopt_options={}
    ipopt_options["max_iter"] = 100
    ipopt_options["print_level"] = 0
    ipopt_options["acceptable_tol"] = 1e-8
    ipopt_options["acceptable_obj_change_tol"] = 1e-6

    opts["ipopt"]=ipopt_options

    solver = nlpsol('solver', 'ipopt', nlp_prob, opts)

    ###############################################################################
    # Execute solver
    args = {}
    args["lbx"] = -float('inf')
    args["ubx"] = float('inf')
    #args["lbg"] = -float('inf')

    args["ubg"] = 250*np.ones(window)
    args["lbg"] = -250*np.ones(window)
    #args["ubg"] = float('inf')
    #args["lbg"] = -float('inf')


    init_state = np.array([[v_init],[p_init]])
    final_state = np.array([[0],[p_ref]])
    u0 = np.zeros((2,window)) # u0 used for initial guess of [vd1, pd1, vd2, pd2, vd3, pd3 ...]


    #init_v_input = np.array([0])
    #args["p"] = np.concatenate((init_state, final_state), axis=None)

    args["p"] =vertcat(init_state, final_state)
    args["x0"] = reshape(u0,2*window,1) #initial guess of [vd1, pd1, vd2, pd2, vd3, pd3 ...]

    sol = solver(lbx=args["lbx"], ubx=args["ubx"], lbg=args["lbg"], ubg=args["ubg"],\
            p=args["p"], x0=args["x0"])

    #CAN_WRITE to make robot move
    mpc_iter = 0
    x_sol = sol['x']

    u = reshape(x_sol,2,window) 

    # record the solved input vd, pd
    u_predichorz_update = np.zeros((1,2,window))
    u_predichorz_update[0,:,:] = u

    # calculate all the state within a predic_horz
    states_predichorz_update = np.zeros((1,2,window+1))
    states_predichorz_update[0,:,:] = ff(u,args["p"]) # get all states within a predic horz
    #print(states_predichorz_update)
    #print(u_predichorz_update)

    # calculate all the v_input within a predic_horz
    VVV = v_input_ff(u,args["p"])
    v_predichorz_update = np.zeros((1,1,window))
    v_predichorz_update[0,:,:] = VVV.T

    np.set_printoptions(precision=2,suppress=True)
    for i in range(1):
        for j in range(window):
            print("----Window Update----")
            print("window: ", j)
            print("state [v, p] = ",states_predichorz_update[i,:,j])
            print("input [vd, pd] = ",u_predichorz_update[i,:,j])
            print("v_input = ",VVV[j])
            print("")
            print("")


    print(x_sol[0:2])

    # here here here


return list([x_sol[0],x_sol[1]])

# ###############################################################################
# # Update the prediction horizontal

#while (abs(args["p"][1]-final_state[1]) > 1e-1):
# for i in range(100):
#     #print("----- Horizontal Prediction Update -----")
#     #shift
#     init_state = states_predichorz_update[mpc_iter,:,1]
#     final_state = np.array([[0],[p_ref]])
#     u0 = u #u0 used for initial guess of [vd1, pd1, vd2, pd2, vd3, pd3 ...]
#     mpc_iter = mpc_iter+1
#     #print("MPC Iteration: ", mpc_iter)
    
#     args["p"] =vertcat(init_state, final_state)
#     args["x0"] = reshape(u0,2*window,1) #initial guess of [vd1, pd1, vd2, pd2, vd3, pd3 ...]

#     sol = solver(lbx=args["lbx"], ubx=args["ubx"], lbg=args["lbg"], ubg=args["ubg"],\
#             p=args["p"], x0=args["x0"])
    
    
    # #CAN_WRITE to make robot move
    # x_sol = sol['x']
    # u = reshape(x_sol,2,window) 
    
    # # record the solved input vd, pd
    # u_predichorz_update= np.concatenate([u_predichorz_update,np.reshape(u,(1,2,window))],0)
        
    # # calculate all the state within a predic_horz
    # states_through_predihorz = ff(u,args["p"]) # get all states within a predic horz
    # # when update a horizontal, add a new page in states_predichorz_update
    # states_predichorz_update = np.concatenate([states_predichorz_update,np.reshape(states_through_predihorz,(1,2,window+1))],0)
    
    # VVV = v_input_ff(u,args["p"])
    # v_predichorz_update = np.concatenate([v_predichorz_update,np.reshape(VVV,(1,1,window))],0)



    # print(states_predichorz_update)
    # print(u_predichorz_update)

    # np.set_printoptions(precision=2,suppress=True)
    # for i in range(1):
    #     for j in range(window):
    #         print("----Window Update----")
    #         print("window: ", j)
    #         print("state [v, p] = ",states_predichorz_update[i,:,j])
    #         print("input [vd, pd] = ",u_predichorz_update[i,:,j])
    #         print("v_input = ",VVV[j])
    #         print("")
    #         print("")

# np.set_printoptions(precision=2,suppress=True)
# for i in range(mpc_iter+1):
#     print("----Prediction Horizontal Update----")
#     print("mpc_iter: ", i)
#     print("state [v, p] = ",states_predichorz_update[i,:,0])
#     print("demand [vd, pd] = ",u_predichorz_update[i,:,0])
#     print("v_input = ",v_predichorz_update[i,0,0])
#     print("")
#     print("")
    