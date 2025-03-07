import random
import pickle
import csv

class QLearn:
    def __init__(self, actions, epsilon, alpha, gamma, filename='current_best_policy'):
        self.q = {}
        self.epsilon = epsilon  # exploration constant
        self.alpha = alpha      # discount constant
        self.gamma = gamma      # discount factor
        self.actions = actions
        self.filename = filename

        self.highest_reward = float('-inf')

    def loadQ(self, filename):
        '''
        Load the Q state-action values from a pickle file.
        '''
        if not filename.endswith('.pickle'):
            filename += '.pickle'
        # TODO: Implement loading Q values from pickle file.
        with open(filename, 'rb') as file:
            self.q = pickle.load(file)

        print("Loaded file: {}".format(filename+".pickle"))

    def saveQ(self, filename):
        '''
        Save the Q state-action values in a pickle file.
        '''
        # TODO: Implement saving Q values to pickle and CSV files.
        if not filename.endswith('.pickle'):
            filename += '.pickle'
        with open(filename, 'wb') as file:
            pickle.dump(self.q, file, protocol=pickle.HIGHEST_PROTOCOL)
        print("Wrote to file: {}".format(filename+".pickle"))


    def getQ(self, state, action):
        '''
        @brief returns the state, action Q value or 0.0 if the value is 
            missing
        '''
        return self.q.get((state, action), 0.0)

    def chooseAction(self, state, return_q=False):
        '''
        @brief returns a random action epsilon % of the time or the action 
            associated with the largest Q value in (1-epsilon)% of the time
        '''
        # TODO: Implement exploration vs exploitation
        #    if we need to take a random action:
        #       * return a random action
        #    else:
        #       * determine which action has the highest Q value for the state 
        #          we are in.
        #       * address edge cases - what if 2 actions have the same max Q 
        #          value?
        #       * return the action with highest Q value
        #
        # NOTE: if return_q is set to True return (action, q) instead of
        #       just action

        # THE NEXT LINES NEED TO BE MODIFIED TO MATCH THE REQUIREMENTS ABOVE 
        if random.random() < self.epsilon:
            chosen_action = random.choice(self.actions)
        else:
            chosen_action = self.findMaxQAction(state)
        if return_q:
            return chosen_action, self.getQ(state, chosen_action)
        return chosen_action

    def learn(self, state1, action1, reward, state2):
        '''
        @brief updates the Q(state,value) dictionary using the bellman update
            equation
        '''
        # TODO: Implement the Bellman update function:
        #     Q(s1, a1) += alpha * [reward(s1,a1) + gamma* max(Q(s2)) - Q(s1,a1)]
        # 
        # NOTE: address edge cases: i.e. 
        # 
        # Find Q for current (state1, action1)
        # Address edge cases what do we want to do if the [state, action]
        #       is not in our dictionary?
        # Find max(Q) for state2
        # Update Q for (state1, action1) (use discount factor gamma for future 
        #   rewards)

        # THE NEXT LINES NEED TO BE MODIFIED TO MATCH THE REQUIREMENTS ABOVE
        current_q = self.getQ(state1, action1)
        max_next_q = self.getQ(state2, self.findMaxQAction(state2))
        updated_q = current_q + self.alpha * (reward + self.gamma * max_next_q - current_q)
        self.q[(state1,action1)] = updated_q
        if reward > self.highest_reward:
            self.highest_reward = reward
            self.saveQ(self.filename)

    def findMaxQAction(self, state):
        potential_actions = []
        prev_action_q_value = self.getQ(state, self.actions[0])
        potential_actions.append(self.actions[0])

        # cycle through all actions and find which one provided the maximized Q value
        for i in range(1,len(self.actions)):
            cur_action = self.actions[i]
            cur_action_q_value = self.getQ(state, cur_action)

            if cur_action_q_value > prev_action_q_value:
                potential_actions = [cur_action]
                prev_action_q_value = cur_action_q_value

            elif cur_action_q_value == prev_action_q_value:
                potential_actions.append(cur_action)
        
        # if no actions were found
        if self.getQ(state, potential_actions[0]) == 0:
            return random.choice(self.actions)

        # random choice because the whole list will have equal Q values
        return random.choice(potential_actions)