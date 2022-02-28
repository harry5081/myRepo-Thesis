from casadi import *
import numpy as np
import math


def errDynFunction(p_ref, v_ref, p_init, v_init):

    np.set_printoptions(precision=3,suppress=True)
    #ws = 1 # check ws direction and sign

    # p_ref_temp =p_ref
    # v_ref_temp =v_ref
    #print("p_ref: ",p_ref[0])
    #print("v_ref: ",v_ref[0])

    #p_ref=np.array([[5.504,32.72,0],[21.41,61.84,0],[45.97,84.15,0]])
    #p_ref=np.array([[10,0,0],[10,0,0]]) # give phi 90 check radius degree!!!!!!!!!!!!

    #p_ref=np.array([[0,100,math.pi/2]])
    #p_ref=np.array([[100,0,0]])
    #p_ref=repmat(p_ref,(window+1),1)




    #p_ref=np.array([[10,0,math.pi/2],[10,0,math.pi/2],[10,0,math.pi/2]]) # give phi 90 check radius degree!!!!!!!!!!!!
    #p_ref=np.array([[0,10,math.pi],[0,10,math.pi],[0,10,math.pi]]) # give phi 90 check radius degree!!!!!!!!!!!!
    #v_ref=np.array([[0,0,0],[0,0,0]])   # vs, 0, ws
    #v_ref=np.array([[0,0,0]])
    #v_ref=repmat(v_ref,(window+1),1)


    #print("p_init: ",p_init)
    #print("v_init: ",v_init)

    #p_init=np.array([10,190,3.1]) # x, y, phi_p
    #v_init=np.array([0,0,0]) # vp, 0, wp

    #p_ref=np.array([[8,191,3.12],[4,200,3.15],[-1,198,3.19],[-1,198,3.19],[-1,198,3.19]]) # give phi 90 check radius degree!!!!!!!!!!!!
    #v_ref=np.array([[50,0,0.25],[50,0,0.25],[50,0,0.25],[50,0,0.25],[50,0,0.25]])   # vp, 0, wp

    # p_init=np.array([0,0,0]) # x, y, phi_p
    # v_init=np.array([0,0,0]) # vp, 0, wp


    p_ref = np.array(p_ref)
    v_ref = np.array(v_ref)

    #p_init= np.array(p_init)
    #v_init= np.array(v_init)

    e_init= p_init-p_ref[0]
    e_init[2]=0

    window = len(p_ref)-1
    dt = 0.2


    acc_max = 1000 #(mm^2/s)



    #############################################################################
    # System Define

    ### Ref, known variables
    xs = SX.sym('xs')
    ys = SX.sym('ys')
    phi_ps = SX.sym('phi_ps')

    vs = SX.sym('vs')
    blanck_s = SX.sym('blanck_s')
    ws = SX.sym('ws')

    ref = vertcat(xs, ys, phi_ps, vs, blanck_s, ws)
    n_ref = ref.shape[0]



    e_x = SX.sym('e_x')
    e_y = SX.sym('e_y')
    e_phi = SX.sym('e_phi')
    e_state = vertcat(e_x, e_y, e_phi)
    n_e_state = e_state.shape[0]

    desire_e_phi = SX.sym('desire_e_phi')


    ### Demand State, output of mpc
    demand_x = SX.sym('demand_x')
    demand_y = SX.sym('demand_y')
    demand_phi_p = SX.sym('demand_phi_p')

    demand_vp = SX.sym('demand_vp')
    demand_blanck = SX.sym('demand_blanck')
    demand_wp = SX.sym('demand_wp')

    demand_state = vertcat(demand_x, demand_y, demand_phi_p, demand_vp, demand_blanck, demand_wp)
    n_demand_state = demand_state.shape[0]



    ### Error dyn function
    T = SX.zeros(3,3)   # check ws direction and sign!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    T[1,0] = -ws        # ws should be sent in as reference in parameter P
    T[0,1] = ws

    Rz = vertcat(
        horzcat(cos(e_phi), -sin(e_phi), 0),
        horzcat(sin(e_phi),  cos(e_phi), 0),
        horzcat(         0,           0, 1)
    )

    err_dyn = T @ e_state + (Rz @ demand_state[3:6]) - ref[3:6]
    err_dyn_f = Function('err_dyn_f',[e_state, demand_state[3:6],ref[3:6]],[err_dyn])


    #############################################################################
    # Casadi Parameters

    P = SX.sym('P',n_demand_state + (window+1)*n_ref) # initial state and reference of the robot
    E = SX.sym('E',n_e_state,window+1) # error states, final value of obj, as low as possible.
    D = SX.sym('D',n_demand_state,window) # future demanded states, solution of obj, solved by casadi


    ACC= SX.sym('ACC',1,window)
    E_Matrix= SX.sym('E_Matrix',3,window)
    #REF_Matrix= SX.sym('REF_Matrix',6,window+1)

    #############################################################################
    # Obj Function

    obj = 0
    g=[]
    g_e=[]
    g_d=[]
    g3=[]

    # Q0 = np.zeros((3,3))
    # Q0[0,0]=1.5    # ex
    # Q0[1,1]=0.5   # ey
    # Q0[2,2]=2    # e_phi


    Q = np.zeros((3,3))
    Q[0,0]=5    # ex
    Q[1,1]=5    # ey
    Q[2,2]=100   # e_phi

    state_init = P[0:6]
    ref_init = P[6:12]

    #REF_Matrix[:,0] = ref_init

    err_init = e_init #(state_init[0:3]-ref_init[0:3])

    #err_init[2]=0
    #state_init[2]=0
    #err_init = err_init + 0.2*err_dyn_f(err_init,state_init,ref_init)
    #obj = err_init.T @ Q0 @ err_init   # this is a problem code
    g_e = vertcat(g_e,E[:,0]-err_init)
    #obj = (err_init.T @ Q @ err_init)
    # err_cur=err_init

    fspeed_init = state_init[3]      # for acc constraint
    fspeed_temp = fspeed_init           # for acc constraint


    for i in range(0,window):
        
        ref_pre = P[n_demand_state + (i)*n_ref : n_demand_state + (i+1)*n_ref]
        ref_current = P[n_demand_state + (i+1)*n_ref : n_demand_state + (i+2)*n_ref]
        demand_cur = D[:,i]
        err_cur = E[:,i]

        err_next =  err_cur + dt * err_dyn_f(err_cur,demand_cur[3:6],ref_pre[3:6])# get vp

        obj=obj+(err_next.T @ Q @ err_next) + 10*demand_cur[3]**2# + (demand_cur[3]-ref_current[3])**2     #5*demand_cur[3]**2#+ 10*(demand_cur[5]-ref_current[5])**2
        
        g_e = vertcat(g_e,E[:,i+1]-err_next)
        g_d = vertcat(g_d,demand_cur[0:3]-ref_current[0:3]-err_next[0:3])  # derive x, y, phi
        

        acc = (D[:,i][3]-fspeed_temp)/dt
        ACC[:,i] = acc
        fspeed_temp = D[:,i][3]

        

    g = vertcat(g,g_e)
    g = vertcat(g,g_d)
    g = vertcat(g,reshape(ACC,window,1))


    ###############################################################################
    # Construct NLP solver
    D_STATE_variables = reshape(D,n_demand_state*window,1) #[x1, y1, phi1, v1, 0, w1, x2, y2, phi2, v2, 0, w2]
    ERROR_variables = reshape(E,n_e_state*(window+1),1) #[ex1, ey1, e_phi1, ex2, ey2, e_phi2...]
    OPT_variables = vertcat(D_STATE_variables,ERROR_variables)
    #OPT_variables = vertcat(ERROR_variables,D_STATE_variables)

    nlp_prob = {'f':obj, 'x':OPT_variables, 'g':g, 'p':P}   #dict

    # Options For Solver
    #opts = {"print_time": False, "ipopt.max_iter": 10}
    opts = {}
    opts["print_time"] = False

    ipopt_options={}
    ipopt_options["max_iter"] = 10000
    ipopt_options["print_level"] = 0
    ipopt_options["acceptable_tol"] = 1e-8
    ipopt_options["acceptable_obj_change_tol"] = 1e-6
    opts["ipopt"]=ipopt_options

    args = {}
    #decision variables (demand state)

    temp_ubx1=float('inf')*np.ones(6)

    #temp_ubx1[2]=math.pi                # demand_phi
    #temp_ubx1[2]=4*math.pi              # demand_phi
    temp_ubx1[3]=150                      # fspeed <150
    temp_ubx1[4]=0                      # blank
    temp_ubx1=repmat(temp_ubx1,window)
    
    temp_ubx2=float('inf')*np.ones(3) # err state
    temp_ubx2[2]=math.pi/2              # constraint e_phi
    temp_ubx2=repmat(temp_ubx2,window+1)
    au=vertcat(temp_ubx1,temp_ubx2)
    #au=vertcat(temp_ubx2,temp_ubx1)

    temp_lbx1=-float('inf')*np.ones(6)
    #temp_lbx1[2]=(-4)*math.pi           # demand_phi
    #temp_lbx1[2]=0                     # demand_phi
    temp_lbx1[3]=-150                      # fspeed >0
    temp_lbx1[4]=0                      # blank
    temp_lbx1=repmat(temp_lbx1,window)

    temp_lbx2=-float('inf')*np.ones(3)  # err state
    temp_ubx2[2]=-math.pi/2              # constraint e_phi
    temp_lbx2=repmat(temp_lbx2,window+1)
    al=vertcat(temp_lbx1,temp_lbx2)
    #al=vertcat(temp_lbx2,temp_lbx1)

    args["lbx"] = al
    args["ubx"] = au

    args["lbg"] = vertcat(np.zeros(g_e.shape[0]+g_d.shape[0]),(-1)*repmat(acc_max,(window),1))
    #np.zeros(n_e_state*(window+1))
    args["ubg"] = vertcat(np.zeros(g_e.shape[0]+g_d.shape[0]),repmat(acc_max,(window),1))
    #np.zeros(n_e_state*(window+1))


    #np.zeros(n_e_state*(window+1))


    #vertcat(np.zeros(6*(window+1)),-1*repmat(v_input_g_temp,(window),1))

    solver = nlpsol('solver', 'ipopt', nlp_prob, opts)

    ###############################################################################
    # Execute solver
    init_state = np.concatenate((p_init,v_init),axis=0)
    #print(init_state)

    #ref_state_temp = np.concatenate((p_ref.T,v_ref.T),axis=0)
    ref_state_temp =np.concatenate((np.reshape(p_ref,(-1,3)),np.reshape(v_ref,(-1,3))),axis=1)
    ref_state = np.reshape(ref_state_temp, newshape=(window+1)*n_ref)
    #print(ref_state)


    #d0 = repmat(init_state,window,1)
    d0 = ref_state[0:6*(window)]
    #d0 = (ref_state[0:6*(window)]+repmat(init_state,window,1))/2
    #u0 = ref_state[n_ref:]  #initial guess of [x1, y1, phi1, v1, 0, w1, ...]

    #init_error = np.array([-10,0,0])


    args["p"] =vertcat(init_state, ref_state)
    #args["x0"] = vertcat(repmat(np.zeros(3),(window+1),1), reshape(d0,6*(window),1))
    args["x0"] = vertcat(reshape(d0,6*(window),1),repmat(np.zeros(3),(window+1),1)) #initial guess of demand state
                                                                                # plus initial guess of error

    sol = solver(lbx=args["lbx"], ubx=args["ubx"], lbg=args["lbg"], ubg=args["ubg"],\
                p=args["p"], x0=args["x0"])


    ###############################################################################
    # Get Solutions
    x_sol = sol['x']
    #print("Solution: ", x_sol)

    error_sol=x_sol[0:3*(window+1)]
    demand_state_sol =x_sol[3*(window+1):3*(window+1) + 6*(window)]

    # demand_state_sol =x_sol[0:6*(window)]
    # error_sol=x_sol[6*(window):6*window + n_e_state*(window+1)]



    #print("Demand state: ",demand_state_sol)
    #print("Error value:",error_sol)


    demand_state = reshape(demand_state_sol,6,window) 
    error = reshape(error_sol,3,window+1) 

    # record the solved state [x1, y1, phi1, v1, 0, w1, ...]
    demand_predichorz_update = np.zeros((1,6,window))
    demand_predichorz_update[0,:,:] = demand_state

    # calculate all the error state within a predic_horz
    err_predichorz_update = np.zeros((1,3,window+1))
    err_predichorz_update[0,:,:] = error # get all err states within a predic horz




    #print("")
    print("ref init    [x, y, phi, v, 0, w] = ",ref_state[0:6])#print("p_init: ",p_init)
    #print("v_init: ",v_init)
    print("state init  [x, y, phi, v, 0, w] = ",init_state)
    print("err init    [ex, ey, e_phi] =      ",err_predichorz_update[0,:,0])
    print("")
    np.set_printoptions(precision=3,suppress=True)
    for i in range(1):
        for j in range(6):
            print("----Window Update----")
            print("window: ", j)
            
            print("ref     [x, y, phi, v, 0, w] = ",ref_state[(j+1)*n_ref : (j+2)*n_ref])
            print("demand  [x, y, phi, v, 0, w] = ",demand_predichorz_update[i,:,j])
            print("err     [ex, ey, e_phi] =      ", err_predichorz_update[i,:,j+1])
            
            #print("v_input = ",v_predichorz_update[i,:,j])
            print("")
            

    return list([demand_state_sol[0],demand_state_sol[1],demand_state_sol[2],demand_state_sol[3],demand_state_sol[4],demand_state_sol[5]])