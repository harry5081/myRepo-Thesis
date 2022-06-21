from casadi import *
import numpy as np
import math


def errDynFunction(p_ref, v_ref, p_init, v_init, Ori_ref, Ori_init, guess, pre_x_sol):
    
    window =len(p_ref)-1
    dt = 0.2
    
    p_ref = np.array(p_ref)
    v_ref = np.array(v_ref)
    p_init= np.array(p_init)
    v_init= np.array(v_init)

    Ori_ref= np.array(Ori_ref)
    Ori_init= np.array(Ori_init)

    guess= np.array(guess)

    vx=v_init[0]*cos(p_init[2])
    vy=v_init[0]*sin(p_init[2])

    e_init_world = p_init-p_ref[0] # world coordinate

    acc_max = 500000000 #(mm^2/s)
   
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

    ### Error in Frenet Frame
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

    # Ref, theta, orientation, known variables
    ori_ref_theta = SX.sym('ori_ref_theta')
    ori_ref_w = SX.sym('ori_ref_w')
    ori_ref = vertcat(ori_ref_theta, ori_ref_w)
    n_ori_ref = ori_ref.shape[0]

    
    # Demand, theta, orientation, output of MPC
    ori_demand_theta = SX.sym('ori_demand_theta')
    ori_demand_w = SX.sym('ori_demand_w')
    ori_demand = vertcat(ori_demand_theta, ori_demand_w)
    n_ori_demand = ori_demand.shape[0]

    e_ori = ori_demand[0]-ori_ref[0]
    n_e_ori = e_ori.shape[0]


    ### Error dyn function
    T = SX.zeros(3,3)   # check ws direction and sign!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    T[1,0] = -ws        # ws should be sent in as reference in parameter P
    T[0,1] = ws


    worldToFrenet = vertcat(
                    horzcat(cos(phi_ps),   sin(phi_ps),  0),
                    horzcat(-sin(phi_ps),    cos(phi_ps),  0),
                    horzcat( 0,             0,            1)
    )
    worldToFrenet_f = Function('worldToFrenet_f',[phi_ps],[worldToFrenet])

    FrenetToworld = vertcat(
                    horzcat(cos(phi_ps),   -sin(phi_ps),  0),
                    horzcat(sin(phi_ps),  cos(phi_ps),  0),
                    horzcat( 0,             0,            1)
    )
    
    FrenetToworld_f = Function('FrenetToworld_f',[phi_ps],[FrenetToworld])
    
    Rz = vertcat(
        horzcat(cos(desire_e_phi), 0, 0),
        horzcat(sin(desire_e_phi), 0, 0),
        horzcat(         0,           0, 1)
    )

    err_dyn = T @ e_state + (Rz @ demand_state[3:6]) - ref[3:6]
    err_dyn_f = Function('err_dyn_f',[e_state, desire_e_phi, demand_state[3:6],ref[3:6]],[err_dyn])

    # Theta dyn
    ori_dyn = ori_demand_w # - ori_ref_w * vs
    ori_dyn_f = Function('ori_dyn_f',[ori_demand_w],[ori_dyn])

    #############################################################################
    # Casadi Parameters

    D = SX.sym('D',n_demand_state,window) # future demanded states, solution of obj, solved by casadi
    P = SX.sym('P',n_demand_state + (window+1)*n_ref + n_ori_demand + (window+1)*n_ori_ref) # initial state and reference of the robot
    E = SX.sym('E',n_e_state,window+1) # error states, final value of obj, as low as possible.

    T_D = SX.sym('T_D',n_ori_demand,window)
    T_E = SX.sym('T_E',n_e_ori,window+1)

    ACC= SX.sym('ACC',1,window)
    ACC2= SX.sym('ACC2',1,window)
    E_Matrix= SX.sym('E_Matrix',3,window)
    REF_Matrix= SX.sym('REF_Matrix',6,window+1)

    #############################################################################
    # Obj Function
    obj = 0
    g=[]
    g_d=[]
    g_e=[]
    
    g_ori_d=[]
    g_ori_e=[]

    state_init = P[0:6]
    ref_init = P[6:12]

    REF_Matrix[:,0] = ref_init

    e_init_Frenet = worldToFrenet_f(ref_init[2]) @ e_init_world # Frenet coordinate

    fspeed_init = state_init[3]      # for acc constraint

    # Orientation
    startOfOri = 6*(window+2)
    ori_init = P[startOfOri:startOfOri + n_ori_ref]
    ori_ref_init = P[startOfOri + n_ori_ref : startOfOri + 2*n_ori_ref]
    ori_err_init = ori_init[0]-ori_ref_init[0]
    g_ori_e = vertcat(g_ori_e,T_E[:,0]-ori_err_init)

    Q = np.zeros((3,3))
    Q[0,0]=5    # ex
    Q[1,1]=5    # ey
    Q[2,2]=1    #100000#100000#100000#35000   # e_phi

    g_e = vertcat(g_e,E[:,0][0:3]-e_init_Frenet[0:3])

    for i in range(window):
        
        ref_pre = P[n_demand_state + (i)*n_ref : n_demand_state + (i+1)*n_ref]
        ref_current = P[n_demand_state + (i+1)*n_ref : n_demand_state + (i+2)*n_ref]
        demand_cur = D[:,i]
        err_cur = E[:,i]
        
        err_next =  err_cur + dt * err_dyn_f(err_cur,E[:,i+1][2],demand_cur[3:6],ref_pre[3:6])# get vp
        #err_next =  err_cur + dt * err_dyn_f(err_cur,demand_cur,ref_pre)
        
        # ori
        ori_ref_current = P[startOfOri + (i+1)*n_ori_ref : startOfOri + (i+2)*n_ori_ref]
        ori_demand_cur = T_D[:,i]
        ori_err_cur = T_E[:,i]
        ori_err_next= ori_err_cur + dt * ori_dyn_f(ori_demand_cur[1])

        #acc = (D[:,i][3]-fspeed_temp)/dt
        #ACC[:,i] = acc
        #fspeed_temp = D[:,i][3]

        #acc = (D[3,i]-fspeed_temp)/dt
        #ACC[:,i] = acc
        #fspeed_temp = D[3,i]

        acc_x = (demand_cur[3]*cos(demand_cur[2])-vx)/dt
        acc_y = (demand_cur[3]*sin(demand_cur[2])-vy)/dt
        ACC[:,i] = acc_x
        ACC2[:,i] = acc_y
        vx = demand_cur[3]*cos(demand_cur[2])
        vy = demand_cur[3]*sin(demand_cur[2])

        #acc = (demand_cur[3]**2+fspeed_temp[1]**2-2*demand_cur[3]*fspeed_temp[1]*cos(demand_cur[2]-fspeed_temp[0]))
        #acc = ((vx- demand_cur[3]*cos(demand_cur[2]))**2+(vy- demand_cur[3]*sin(demand_cur[2]))**2)/dt
        #ACC[:,i] = acc
        #vx = demand_cur[3]*cos(demand_cur[2])
        #vy = demand_cur[3]*sin(demand_cur[2])

        #fspeed_temp[0] = demand_cur[2]
        #fspeed_temp[1] = demand_cur[3]

        #obj=obj+10*(err_next.T @ Q @ err_next) + (window-i)**2*demand_cur[3]**2 + (ori_err_next**2 + 0.5*ori_demand_cur[1]**2)# + 0.3*ACC[:,i]**2 + 0.1*(ori_err_next**2 + 5*ori_demand_cur[1]**2)
        obj=obj+1*(err_next.T @ Q @ err_next) + 1*demand_cur[3]**2 + (ori_err_next**2 + 1.5*ori_demand_cur[1]**2)# + 0.3*ACC[:,i]**2 + 0.1*(ori_err_next**2 + 5*ori_demand_cur[1]**2)
        
        # pos vel
        g_e = vertcat(g_e,E[:,i+1][0:3]-err_next[0:3])
        g_d = vertcat(g_d,demand_cur[0:3]-ref_current[0:3]- FrenetToworld_f(ref_current[2]) @ err_next[0:3])  # derive x and y        
        
        # ori
        g_ori_d = vertcat(g_ori_d,ori_demand_cur[0]-ori_ref_current[0]-ori_err_next)
        g_ori_e = vertcat(g_ori_e,T_E[:,i+1]-ori_err_next)
        

    g = vertcat(g,g_e)
    g = vertcat(g,g_d)

    g = vertcat(g,reshape(ACC,window,1))
    g = vertcat(g,reshape(ACC2,window,1))


    g = vertcat(g,g_ori_d)
    g = vertcat(g,g_ori_e)


    ###############################################################################
    # Construct NLP solver
    D_STATE_variables = reshape(D,n_demand_state*window,1) #[x1, y1, psi1, v1, 0, w1, x2, y2, psi2, v2, 0, w2]
    ERROR_variables = reshape(E,n_e_state*(window+1),1) #[ex1, ey1, e_psi1, ex2, ey2, e_psi2...]

    D_ORI_variables = reshape(T_D,n_ori_demand*window,1) #[theta1, w1, theta2, w2]
    ERROR_ORI_variables = reshape(T_E,n_e_ori*(window+1),1)

    OPT_variables = vertcat(ERROR_variables,D_STATE_variables,D_ORI_variables,ERROR_ORI_variables)

    nlp_prob = {'f':obj, 'x':OPT_variables, 'g':g, 'p':P}   #dict

    # Options For Solver
    opts = {}
    opts["print_time"] = False

    ipopt_options={}
    ipopt_options["max_iter"] = 100#2000
    ipopt_options["print_level"] = 0
    ipopt_options["acceptable_tol"] = 1e-8
    ipopt_options["acceptable_obj_change_tol"] = 1e-6
    opts["ipopt"]=ipopt_options


    args = {}
    #--------------------------------------------------
    ### decision variables (demand state)
    # desired states_max
    temp_ubx1=float('inf')*np.ones(6)
    #temp_ubx1[2]=math.pi                # demand_phi
    temp_ubx1[2]=2*math.pi              # demand_phi
    temp_ubx1[3]=250                      # fspeed <150
    temp_ubx1[4]=0                      # blank
    temp_ubx1=repmat(temp_ubx1,window)

    # Frenet frame_max
    temp_ubx2=float('inf')*np.ones(3) # err state
    temp_ubx2[2]=1*math.pi
    temp_ubx2=repmat(temp_ubx2,window+1)

    # ori_max
    temp_ubx3=float('inf')*np.ones(2) # demand ori
    temp_ubx3[1]=30
    temp_ubx3=repmat(temp_ubx3,window)
    
    temp_ubx4=float('inf')*np.ones(1) # err ori
    temp_ubx4=repmat(temp_ubx4,window+1)

    au=vertcat(temp_ubx2,temp_ubx1,temp_ubx3,temp_ubx4)

    #--------------------------------------------------
    # desired states_min
    temp_lbx1=-float('inf')*np.ones(6)
    temp_lbx1[2]=(-2)*math.pi            # demand_phi
    #temp_lbx1[2]=0                     # demand_phi
    temp_lbx1[3]=0                     # fspeed >0
    temp_lbx1[4]=0                      # blank
    temp_lbx1=repmat(temp_lbx1,window)

    # Frenet frame_min
    temp_lbx2=-float('inf')*np.ones(3)  # err state
    temp_lbx2[2]=(-1)*math.pi
    temp_lbx2=repmat(temp_lbx2,window+1)

    # ori_min
    temp_lbx3=-float('inf')*np.ones(2) # demand ori
    temp_lbx3[1]=-30
    temp_lbx3=repmat(temp_lbx3,window)
    
    temp_lbx4=-float('inf')*np.ones(1) # err ori
    temp_lbx4=repmat(temp_lbx4,window+1)

    al=vertcat(temp_lbx2,temp_lbx1,temp_lbx3,temp_lbx4)

    args["lbx"] = al
    args["ubx"] = au
    
    #args["lbg"] = vertcat(np.zeros(3*(window)+g_e.shape[0]),repmat(-1*acc_max,(window),1),np.zeros(g_ori_d.shape[0]),np.zeros(g_ori_e.shape[0]))
    #args["ubg"] = vertcat(np.zeros(3*(window)+g_e.shape[0]),repmat(acc_max,(window),1),np.zeros(g_ori_d.shape[0]),np.zeros(g_ori_e.shape[0]))
    args["lbg"] = vertcat(np.zeros(3*(window)+g_e.shape[0]),repmat(-1*acc_max,(window),1),repmat(-1*acc_max,(window),1),np.zeros(g_ori_d.shape[0]),np.zeros(g_ori_e.shape[0]))
    args["ubg"] = vertcat(np.zeros(3*(window)+g_e.shape[0]),repmat(acc_max,(window),1),repmat(acc_max,(window),1),np.zeros(g_ori_d.shape[0]),np.zeros(g_ori_e.shape[0]))
   
    solver = nlpsol('solver', 'ipopt', nlp_prob, opts)

    ###############################################################################
    # Execute solver
    init_state = np.concatenate((p_init,v_init),axis=0)
    
    ref_state_temp =np.concatenate((np.reshape(p_ref,(-1,3)),np.reshape(v_ref,(-1,3))),axis=1)
    ref_state = np.reshape(ref_state_temp, newshape=(window+1)*n_ref)

    Ori_ref = np.reshape(Ori_ref, newshape=(window+1)*n_ori_ref)

    # initial guess ----------------------------------------------------------------   
    #d0 = repmat(init_state,window,1)  #initial states as guess
    #d0 = repmat(pre_sol,window,1)     #first element of previous solution as guess    
    
    theta0 = repmat(Ori_init,window,1)
    
    args["p"] =vertcat(init_state, ref_state, Ori_init, Ori_ref)
    #args["x0"] = vertcat(reshape(d0,6*(window),1),repmat(np.zeros(3),(window+1),1),reshape(theta0,2*(window),1),repmat(np.zeros(1),(window+1),1)) 
                                                                                #initial guess of demand state
                                                                                # plus initial guess of error
    
    guess_temp=np.reshape(guess,(-1,1))
    guess_temp = guess_temp[0:6*(window)]
    
    # reference as guess                                                       
    args["x0"] = vertcat(repmat(np.zeros(3),(window+1),1),guess_temp,reshape(theta0,2*(window),1),repmat(np.zeros(1),(window+1),1)) 
    
    # previous solution as guess 
    #args["x0"] = pre_x_sol
    #----------------------------------------------------------------------------

    sol = solver(lbx=args["lbx"], ubx=args["ubx"], lbg=args["lbg"], ubg=args["ubg"],\
                p=args["p"], x0=args["x0"])


    ###############################################################################
    # Get Solutions
    x_sol = sol['x']
    error_sol=x_sol[0:3*(window+1)]
    
    demand_state_sol =x_sol[3*(window+1):3*(window+1) + 6*(window)]
    
    demand_state = reshape(demand_state_sol,6,window) 
    error = reshape(error_sol,3,window+1) 

    # record the solved desired state [x1, y1, phi1, v1, 0, w1, ...]
    demand_predichorz_update = np.zeros((1,6,window))
    demand_predichorz_update[0,:,:] = demand_state

    # extract all the error state within a predic_horz
    err_predichorz_update = np.zeros((1,3,window+1))
    err_predichorz_update[0,:,:] = error # get all err states within a predic horz

    # Orientation
    startOrisol = 6*window + n_e_state*(window+1)

    demand_ori_sol = x_sol[startOrisol:startOrisol+2*(window)]
    error_ori_sol=x_sol[startOrisol+2*(window):startOrisol+2*(window) + n_e_ori*(window+1)]
   
    demand_ori = reshape(demand_ori_sol,2,window) 
    error_ori = reshape(error_ori_sol,n_e_ori,window+1) 

    # record the solved desired state of ori
    demand_ori_predichorz_update = np.zeros((1,2,window))
    demand_ori_predichorz_update[0,:,:] = demand_ori

    # extract all the error ori within a predic_horz
    err_ori_predichorz_update = np.zeros((1,1,window+1))
    err_ori_predichorz_update[0,:,:] = error_ori # get all ori err within a predic horz

    #print("")
    #print("ref init    [x, y, phi, v, 0, w] = ",ref_state[0:6])#print("p_init: ",p_init)
    #print("state init  [x, y, phi, v, 0, w] = ",init_state)
    #print("err init F  [ex, ey, e_phi] =      ",err_predichorz_update[0,:,0])
    
    # print("*")
    # print("*oriRef init [z, w] =             *",Ori_ref[0:2])
    # print("*ori init    [z, w] =             *",Ori_init)
    # print("*oriErr init [ez] =               *",err_ori_predichorz_update[0,:,0])

    print("")
    np.set_printoptions(precision=6,suppress=True)
    for i in range(1):
        for j in range(0):#(window):
            print("----Window Update----")
            #print("window: ", j)
            #print("err F   [ex, ey, e_phi] =      ", err_predichorz_update[i,:,j+1])
            #print("ref     [x, y, phi, v, 0, w] = ",ref_state[(j+1)*n_ref : (j+2)*n_ref])
            #print("demand  [x, y, phi, v, 0, w] = ",demand_predichorz_update[i,:,j])
            
            #print("v_input = ",v_predichorz_update[i,:,j])
            # print("*")
            # print("*ori_ref [z, w] =             *",Ori_ref[j*n_ori_ref : (j+1)*n_ori_ref])
            # print("*ori_de  [z, w] =             *",demand_ori_predichorz_update[i,:,j])
            # print("*ori_err [ez] =               *",err_ori_predichorz_update[i,:,j+1])
            # print(" ")
    
    return np.array(x_sol)
    