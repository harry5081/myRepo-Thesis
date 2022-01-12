from casadi import *
import numpy as np
 
#def functionTest(v_ref, p_ref, v_init, p_init, v_input_begin):
#def functionTest():

kf=1
kd=0.3
kp=3
ki=0

a = 0.02
t = 0.02

window = 20

v_ref=0
p_ref=200
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

e = exp(-300*t/(fabs(v_input-v)+0.1))
ef = Function('ef',[states,controls],[e])

e2 = exp(-10000*t/((v_input-v)**2+0.1))

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

model2 = SX.sym('model2',2,1)
model2[0] = v_input-(v_input-v)*e2
model2[1] = t*v+p

###########################################################################
# Make mathematical model as function object
f = Function('f',[states,controls],[model])
f2 = Function('f',[states,controls],[model2])


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

    # e_factor = ef(state_current,control_current)
    # A_factor = Af(state_current,control_current)
    # h_factor = hf(state_current,control_current) 
    #state_next = A_factor @ state_current + h_factor @ B @ state_current + h_factor @ C @ control_current
    
    state_next = f(state_current,control_current)
    #state_next = f2(state_current,control_current)


    # e = exp(-200*t/(fabs(V_INPUT_MATRIX[i]-state_current[0])+0.01))
    # A = SX.sym('A',2,2)
    # A[0,0]=e
    # A[0,1]=0
    # A[1,0]=t
    # A[1,1]=1
    
    # h = SX.sym('h',2,1)
    # h[0,0] = 1-e
    # h[1,0] = 0

    #state_next = A @ state_current + h @ B @ state_current + h @ C @ control_current
    

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
    obj = obj + 10*(X[:,i+1] - P[2:4]).T @ Q @ (X[:,i+1] - P[2:4])+ V_INPUT_MATRIX[i].T @ V_INPUT_MATRIX[i]
    #(control.T @ control)+ 3*v_input_dff**2#+(control.T @ control)#+ V_INPUT_MATRIX[i].T @ V_INPUT_MATRIX[i] + 3*v_input_dff**2 #+ 0.01*(control_diff.T @ control_diff)

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
ipopt_options["acceptable_tol"] = 1e-6
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


# print(x_sol[0:2])

# here here here


    # return list([x_sol[0],x_sol[1]])

###############################################################################
# Update the prediction horizontal

#while (abs(args["p"][1]-final_state[1]) > 1):
for i in range(100):
    #print("----- Horizontal Prediction Update -----")
    #shift
    init_state = states_predichorz_update[mpc_iter,:,1]
    final_state = np.array([[0],[p_ref]])
    u0 = u #u0 used for initial guess of [vd1, pd1, vd2, pd2, vd3, pd3 ...]
    mpc_iter = mpc_iter+1
    #print("MPC Iteration: ", mpc_iter)
    
    args["p"] =vertcat(init_state, final_state)
    args["x0"] = reshape(u0,2*window,1) #initial guess of [vd1, pd1, vd2, pd2, vd3, pd3 ...]

    sol = solver(lbx=args["lbx"], ubx=args["ubx"], lbg=args["lbg"], ubg=args["ubg"],\
            p=args["p"], x0=args["x0"])
    
    
    #CAN_WRITE to make robot move
    x_sol = sol['x']
    u = reshape(x_sol,2,window) 
    
    # record the solved input vd, pd
    u_predichorz_update= np.concatenate([u_predichorz_update,np.reshape(u,(1,2,window))],0)
        
    # calculate all the state within a predic_horz
    states_through_predihorz = ff(u,args["p"]) # get all states within a predic horz
    # when update a horizontal, add a new page in states_predichorz_update
    states_predichorz_update = np.concatenate([states_predichorz_update,np.reshape(states_through_predihorz,(1,2,window+1))],0)
    
    VVV = v_input_ff(u,args["p"])
    v_predichorz_update = np.concatenate([v_predichorz_update,np.reshape(VVV,(1,1,window))],0)

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

np.set_printoptions(precision=2,suppress=True)
for i in range(mpc_iter+1):
    print("----Prediction Horizontal Update----")
    print("mpc_iter: ", i)
    print("state [v, p] = ",states_predichorz_update[i,:,0])
    print("demand [vd, pd] = ",u_predichorz_update[i,:,0])
    print("v_input = ",v_predichorz_update[i,0,0])
    print("")
    print("")
    