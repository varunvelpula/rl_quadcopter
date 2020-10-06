import sim
import sys
import torch
import numpy as np
import random
import gym
import custom_gym

sim.simxFinish(-1)
# clientID = sim.simxStart('127.0.0.1',19997,True,True,5000,5)


env = gym.make('RLQuad-v0')  # look into the name of the library

def epsilon_greedy_action(q_value, state, epsilon):
    """
    Implements an epsilon-greedy action selection.
    
    Args:
      q_values (Tensor): Tensor representing the predicted q_values for each action (matrix form)
      epsilon  (Float):  Float from 0-1 representing the probability of selecting a random action, instead of the optimal
      
    """
    prob = random.random()
    if prob < epsilon:
        return np.argmax(q_value[state])
    else:
      return np.random.randint(7) 

''' 
    Q learning implementation
'''

state_size = (5,5,10)
q_values_size = (5,5,10,7) # (state size, action size)
q_values = np.zeros(q_values_size)
reward = np.full(state_size, -1)
final_target = [0,0,0.2]
gamma = 0.9
threshold = 0.01
learning_rate = 0.8
env.reset()

min_eps = 0.01
decay_constant = 100

for i in range(500):
    print(i)
    slope = (min_eps - 1.0) / decay_constant
    epsilon = 1 - max(slope * i + 1.0, min_eps)
    state = env.reset()
    done = False
    while not done:
        action = epsilon_greedy_action(q_values, state, epsilon = 0.1)
        print(action)
        next_state, reward, done, _ = env.step(action)
        td_target = reward + gamma * np.max(q_values[next_state])
        td_error = td_target - q_values[state][action]
        q_values[state][action] += learning_rate * td_error
        # Update state
        state = next_state




# policy iteration
# state_size = (5,5,10)
# V = np.zeros(state_size)
# policy = np.random.randint(4, size=state_size)  
# reward = np.full(state_size, -1)
# final_target = [0,0,0.2]
# gamma = 0.9
# threshold = 0.01
# env.reset()

# i = 0 # iteration
# while True:
#     delta = 0 # To check for convergence
#     for row in range (state_size[0]):
#         for col in range(state_size[1]):
#             old_v = V[row][col]
#             # Choose the action with maximum future reward/value
#             action = choose_action(row, col) 
#             env.S = (row, col)
#             new_state, r, done, info = modified_step(action) 
#             # Bellman equation value update
#             V[row][col] = reward[row][col] + 0.9 * V[new_state[0]][new_state[1]] 
#             # Policy update
#             policy[row][col] = action

#             # Checking for convergence
#             delta = max(delta, np.abs(old_v - V[row][col]))

#     #         break
#     #     break
#     # break
#     i += 1
#     if delta < threshold:
#         print (str(i)+' iterations done, reached convergence!')
#         break    

# state = env.reset()
# done = False
# total_reward = 0
# while done != True: 
#     action = policy[state[0]][state[1]]
#     print(state, action)
#     state, reward, done, info = modified_step(action) #take step using selected action
#     total_reward = total_reward + reward
  
# #Print the reward of these actions
# print("Total Reward: " + str(total_reward)) 



# def choose_action(row, col): 
#     best_action = None
#     best_value = float('-inf')

#     # For each action, take it from the current state, compute the value of the new state using Bellman eqn
#     # Find the action which gives the highest value and return it. 
#     for action in range (6)):
#         env.S = (row, col, height)
#         new_state, r, done, info = modified_step(action) 
#         # print((row, col), env.moves[action], new_state)
#         new_value = gamma * V[new_state[0]][new_state[1]][new_state[2]]
#         # print(new_value)
#         if new_value > best_value:
#             best_value = new_value
#             best_action = action
#     return best_action