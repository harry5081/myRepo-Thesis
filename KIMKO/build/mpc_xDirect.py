from casadi import *
import numpy as np

#def functionTest(v_ref=0, p_ref=100, v_init=0, p_init=0, v_input_begin=0):
def functionTest():

    kf=1
    kd=0.3
    kp=3
    ki=0
    
    a = 0.7
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
    A = SX([[0,0],[0,1]])
    B = SX([[-kd,-kp]])
    C = SX([[kf+kd,kp]])
    h = SX([[a],[t]])

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

    ############################################################################    
    # Calculate in a prediction horizontal
    X[:,0]=P[0:2] # assign the initial state to the robot
    for i in range(window): # fill out all prediction state within a predic horz

        state_current = X[:,i]
        control_current = U[:,i]
        state_next = f(state_current,control_current)
        X[:,i+1] = state_next
        
        V_INPUT_MATRIX[i]= v_input_f(state_current,control_current)

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
        obj = obj + (state - P[2:4]).T @ Q @ (state - P[2:4]) + v_input_dff **2
        
    ###############################################################################
    # Construct NLP solver
    OPT_variables = reshape(U,2*window,1) #[vd1, pd1, vd2, pd2, vd3, pd3 ...]

    g=[]
    #g = V_INPUT_MATRIX
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
    args["lbg"] = -float('inf')

    #args["ubg"] = 300*np.ones(window)
    args["ubg"] = float('inf')


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
    VVV = v_input_ff(u,args["p"])
    #print(states_predichorz_update)
    #print(u_predichorz_update)

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

    
    print(x_sol[0:2])
    
    return list([x_sol[0],x_sol[1]])

    # ###############################################################################
    # # Update the prediction horizontal

    # #while (abs(args["p"][1]-final_state[1]) > 1e-1):
    # for i in range(0):
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
        
        
    #     #CAN_WRITE to make robot move
    #     x_sol = sol['x']
    #     u = reshape(x_sol,2,window) 
        
    #     # record the solved input vd, pd
    #     u_predichorz_update= np.concatenate([u_predichorz_update,np.reshape(u,(1,2,window))],0)
            
    #     # calculate all the state within a predic_horz
    #     states_through_predihorz = ff(u,args["p"]) # get all states within a predic horz
    #     # when update a horizontal, add a new page in states_predichorz_update
    #     states_predichorz_update = np.concatenate([states_predichorz_update,np.reshape(states_through_predihorz,(1,2,window+1))],0)
        
    #     # print(states_predichorz_update)
    #     # print(u_predichorz_update)

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

    # # np.set_printoptions(precision=2,suppress=True)
    # # for i in range(mpc_iter+1):
    # #     print("----Prediction Horizontal Update----")
    # #     print("mpc_iter: ", i)
    # #     print("state [v, p] = ",states_predichorz_update[i,:,0])
    # #     print("input [vd, pd] = ",u_predichorz_update[i,:,0])
    # #     print("")
    # #     print("")
        
