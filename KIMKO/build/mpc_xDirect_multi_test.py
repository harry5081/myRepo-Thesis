from casadi import *
import numpy as np

def functionTest2(v_ref, p_ref, v_init, p_init, v_input_begin, pre_vd, pre_pd):
#def functionTest():

    kf=1
    kd=0.3
    kp=3
    ki=0

    a = 0.02
    t = 0.02

    window = 20

    # v_ref=0
    # p_ref=100
    # v_init=0
    # p_init=0
    # v_input_begin=0

    pre_control = np.array([[pre_vd],[pre_pd]])



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
    #A = SX([[0,0],[0,1]])
    A = SX([[1-a,0],[t,1]])
    B = SX([[-kd,-kp]])
    C = SX([[kf+kd,kp]])
    #h = SX([[a],[t]])
    h = SX([[a],[0]])

    model = A @ states + h @ B @ states + h @ C @ controls
    v_input = B @ states + C @ controls

    ###########################################################################
    # Make mathematical model as function object
    f = Function('f',[states,controls],[model])
    v_input_f = Function('v_input_f',[states,controls],[v_input])

    U = SX.sym('U',n_controls,window) # vd,pd during a window
    P = SX.sym('P',n_state + n_state) # initial state and reference state of the robot
    X = SX.sym('X',n_state,(window+1)) # states during prediction horizontal

    V_INPUT_MATRIX= SX.sym('V_INPUT_MATRIX',window)

    # Construct obj function
    obj = 0
    g=[]

    Q = np.zeros((2,2))
    Q[0,0]=3
    Q[1,1]=0
    #print(Q)

    R = np.zeros((2,2))
    R[0,0]=20
    R[1,1]=20
    #print(R)

    v_input_temp = v_input_begin

    ############################################################################    
    # Calculate in a prediction horizontal

    #X[:,0]=P[0:2] # assign the initial state to the robot
    state_current = X[:,0]
    g = vertcat(g,state_current-P[0:2]) 



    for i in range(window): # fill out all prediction state within a predic horz
        
        state_current = X[:,i]
        control_current = U[:,i]

        control_diff = control_current-pre_control




        V_INPUT_MATRIX[i]= v_input_f(state_current,control_current)
        v_input_dff = V_INPUT_MATRIX[i]- v_input_temp
        v_input_temp = V_INPUT_MATRIX[i]

        #obj = obj + 1.5*i*(X[:,i+1] - P[2:4]).T @ Q @ (X[:,i+1] - P[2:4]) + V_INPUT_MATRIX[i] **2
        obj = obj + 10*(X[:,i+1] - P[2:4]).T @ Q @ (X[:,i+1] - P[2:4]) + control_current.T @ R @control_current+ V_INPUT_MATRIX[i] **2 + 0.001*(control_diff.T @ control_diff)

        state_next_multi_shoot = X[:,i+1]

        state_next = f(state_current,control_current)
        g = vertcat(g,state_next_multi_shoot-state_next)
        #X[:,i+1] = state_next
        
    v_input_ff = Function('v_input_ff',[U,X],[V_INPUT_MATRIX])
        
    ###############################################################################
    # Construct NLP solver
    STATE_variables = reshape(X,2*(window+1),1) #[X0, X1, X2, X3...]
    INPUT_variables = reshape(U,2*window,1) #[vd1, pd1, vd2, pd2, vd3, pd3 ...]
    OPT_variables = vertcat(INPUT_variables,STATE_variables)
    #print(OPT_variables)

    g = vertcat(g,V_INPUT_MATRIX)

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
    args["lbg"] = vertcat(np.zeros(2*(window+1)),-250*np.ones(window))
    args["ubg"] = vertcat(np.zeros(2*(window+1)),250*np.ones(window))


    solver = nlpsol('solver', 'ipopt', nlp_prob, opts)

    init_state = np.array([[v_init],[p_init]])
    final_state = np.array([[v_ref],[p_ref]])
    
    #u0 = np.zeros((2,window)) # u0 used for initial guess of [vd1, pd1, vd2, pd2, vd3, pd3 ...]
    
    u0 = repmat(pre_control,window,1)
    
    #u0 = repmat(final_state,window,1)
    
    #u0 = repmat(init_state,window,1)


    #init_v_input = np.array([0])
    #args["p"] = np.concatenate((init_state, final_state), axis=None)

    state_0_temp = repmat(init_state,(window+1),1)

    args["p"] =vertcat(init_state, final_state)
    args["x0"] = vertcat(reshape(u0,2*window,1),reshape(state_0_temp,2*(window+1),1)) #initial guess of [vd1, pd1, vd2, pd2, vd3, pd3 ...]
    #args["x0"] = vertcat(reshape(u0,2*window,1),repmat(init_state,(window+1),1)) #initial guess of [vd1, pd1, vd2, pd2, vd3, pd3 ...]
    #args["x0"] = vertcat(reshape(u0,2*window,1),np.zeros(2*(window+1))) #initial guess of [vd1, pd1, vd2, pd2, vd3, pd3 ...]



    sol = solver(lbx=args["lbx"], ubx=args["ubx"], lbg=args["lbg"], ubg=args["ubg"],\
            p=args["p"], x0=args["x0"])



    #CAN_WRITE to make robot move
    mpc_iter = 0
    x_sol = sol['x']

    #print(x_sol)

    u_sol =x_sol[0:2*(window)]
    state_sol=x_sol[2*(window):2*(window+1)+2*window]

    #print(u_sol)
    #print(state_sol)

    u = reshape(u_sol,2,window) 
    state = reshape(state_sol,2,window+1) 

    # record the solved input vd, pd
    u_predichorz_update = np.zeros((1,2,window))
    u_predichorz_update[0,:,:] = u

    # calculate all the state within a predic_horz
    states_predichorz_update = np.zeros((1,2,window+1))
    states_predichorz_update[0,:,:] = state # get all states within a predic horz
    #print(states_predichorz_update)
    #print(u_predichorz_update)

    # calculate all the v_input within a predic_horz
    VVV = v_input_ff(u,state)
    v_predichorz_update = np.zeros((1,1,window))
    v_predichorz_update[0,:,:] = VVV.T

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


    return list([u_sol[0],u_sol[1]])
# print(x_sol[0:2])



###############################################################################
# Update the prediction horizontal

#while (abs(args["p"][1]-final_state[1]) > 1e-1):
# for i in range(100):
#     #print("----- Horizontal Prediction Update -----")
#     #shift
#     init_state = states_predichorz_update[mpc_iter,:,1]
#     final_state = np.array([[0],[p_ref]])
#     state_0 = state_sol
#     u0 = u_sol #u0 used for initial guess of [vd1, pd1, vd2, pd2, vd3, pd3 ...]
#     mpc_iter = mpc_iter+1
#     #print("MPC Iteration: ", mpc_iter)
    
#     args["p"] =vertcat(init_state, final_state)
#     #args["x0"] = reshape(u0,2*window,1) #initial guess of [vd1, pd1, vd2, pd2, vd3, pd3 ...]
#     args["x0"] =vertcat(reshape(u0,2*window,1),repmat(init_state,(window+1),1)) #initial guess of [vd1, pd1, vd2, pd2, vd3, pd3 ...]
#     #args["x0"] = vertcat(reshape(u0,2*window,1),reshape(state_0,2*(window+1),1)) #initial guess of [vd1, pd1, vd2, pd2, vd3, pd3 ...]

#     sol = solver(lbx=args["lbx"], ubx=args["ubx"], lbg=args["lbg"], ubg=args["ubg"],\
#             p=args["p"], x0=args["x0"])
    
    
#     #CAN_WRITE to make robot move
#     x_sol = sol['x']

#     u_sol =x_sol[0:2*(window)]
#     state_sol=x_sol[2*(window):2*(window+1)+2*window]

#     u = reshape(u_sol,2,window) 
#     state = reshape(state_sol,2,window+1) 
    
    
#     # record the solved input vd, pd
#     u_predichorz_update= np.concatenate([u_predichorz_update,np.reshape(u,(1,2,window))],0)
        
#     # calculate all the state within a predic_horz
#     states_through_predihorz = state # get all states within a predic horz
#     # when update a horizontal, add a new page in states_predichorz_update
#     states_predichorz_update = np.concatenate([states_predichorz_update,np.reshape(states_through_predihorz,(1,2,window+1))],0)
    
#     VVV = v_input_ff(u,state)
#     v_predichorz_update = np.concatenate([v_predichorz_update,np.reshape(VVV,(1,1,window))],0)



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

#     np.set_printoptions(precision=2,suppress=True)
#     for i in range(mpc_iter+1):
#         print("----Prediction Horizontal Update----")
#         print("mpc_iter: ", i)
#         print("state [v, p] = ",states_predichorz_update[i,:,0])
#         print("demand [vd, pd] = ",u_predichorz_update[i,:,0])
#         print("v_input = ",v_predichorz_update[i,0,0])
#         print("")
#         print("")

     
            
