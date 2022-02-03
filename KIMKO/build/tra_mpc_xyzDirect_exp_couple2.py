from casadi import *
import numpy as np
import math

def functionTest(v_ref, p_ref, v_init, p_init, v_input_begin, pre_vd_pd):

 

    return list([0,0,0,0,0,0])
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

    
            