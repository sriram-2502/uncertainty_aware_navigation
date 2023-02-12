# ageny.py
# ---------
# Licensing Information:  You are free to use or extend these projects for
# educational purposes provided that (1) you do not distribute or publish
# solutions, (2) you retain this notice, and (3) you provide clear
# attribution to Clemson University and the author.
# 
# Author: Ioannis Karamouzas (ioannis@g.clemson.edu)
#

import numpy as np
from abc import ABC, abstractmethod

""" 
    Abstract class for agents
"""
class AbstractAgent(ABC):

    def __init__(self, inputParameters):
        """ 
            Takes an input line from the csv file,  
            and initializes the agent
        """
        self.id = int(inputParameters[0]) # the id of the agent
        self.gid = int(inputParameters[1]) # the group id of the agent
        self.pos = np.array([float(inputParameters[2]), float(inputParameters[3])]) # the position of the agent 
        self.vel = np.zeros(2) # the velocity of the agent
        self.goal = np.array([float(inputParameters[4]), float(inputParameters[5])]) # the goal of the agent
        self.prefspeed = float(inputParameters[6]) # the preferred speed of the agent
        self.gvel = self.goal-self.pos # the goal velocity of the agent
        self.gvel = self.gvel/(np.sqrt(self.gvel.dot(self.gvel )))*self.prefspeed       
        self.maxspeed = float(inputParameters[7]) # the maximum sped of the agent
        self.radius = float(inputParameters[8]) # the radius of the agent
        self.atGoal = False # has the agent reached its goal?
     
    @abstractmethod
    def computeAction(self, neighbors=[]):
        """
            Performs a sense and act simulation step.
        """
        pass

    @abstractmethod    
    def update(self, dt):
        """
            Updates the state of the character, given the time step of the simulation.
        """
        pass

""" 
    Agent class that implements the 2000 Social Force Model by Helbing et al.
    See the paper for details  
"""
class SFMAgent(AbstractAgent):

    def __init__(self, inputParameters, goalRadius=1, dhor = 10, ksi=0.5, A=2000, B=0.08, k=1.2e5, kappa=2.4e5, mass = 80, maxF = 10):
        """ 
           Initializes the agent
        """
        super().__init__(inputParameters)
        self.atGoal = False # has the agent reached its goal?
        self.goalRadiusSq = goalRadius*goalRadius # parameter to determine if agent is close to the goal
        self.dhor = dhor # the sensing radius
        self.ksi = ksi # the relaxation time used to compute the goal force
        self.A = A # scaling constant for the repulsive force
        self.B = B # safe distance that the agent prefers to keep
        self.k = k # scaling constant of the body force in case of a collision
        self.kappa = kappa # scaling constant of the sliding friction foce in case of a collision
        self.F = np.zeros(2) # the total force acting on the agent
        self.mass = 80 # mass of the agent
        self.maxF = self.mass*maxF # the maximum force that can be applied to the agent


    def computeAction(self, neighbors=[]):       
        if not self.atGoal:
            f_goal = self.mass*(self.gvel - self.vel)/self.ksi
            # compute repulsive force for each neighbor within sensing radius
            f_avoid = 0
            for n in neighbors:
                # remove self when computing neighbors
                if n.id != self.id:
                    d_relative = np.linalg.norm(self.pos - n.pos) - (self.radius + n.radius)
                    
                    # if neighbors are inside sensing radius
                    if d_relative < self.dhor:
                        x_i = self.pos
                        x_j = n.pos
                        v_i = self.vel
                        v_j = n.vel

                        # get relative position
                        d_ij = np.linalg.norm(x_i-x_j) # distance between center of mass
                        r_ij = self.radius + n.radius # sum of agent radii
                        
                        # get tangential and normal vectors
                        n_ij = (x_i-x_j)/d_ij # normalized direction vector
                        t_ij = np.array(-n_ij[1],n_ij[0]) # tangent vector

                        # get tangential velocity difference
                        v_ij = np.dot((v_i-v_j),t_ij)

                        # compute g function
                        g = lambda x : max(r_ij-d_ij,0) 

                        # compute avoidance forces
                        f_repulsive = self.A*np.exp((r_ij-d_ij)/self.B)
                        f_body = self.k*g(r_ij-d_ij)
                        f_friction = self.kappa*g(r_ij-d_ij)*v_ij
                        f_avoid = f_avoid + (f_repulsive+f_body)*n_ij + f_friction*t_ij
                    
                    else:
                        f_avoid = f_avoid + 0
            
            # compute final force
            self.F =  f_goal + f_avoid
            if(np.linalg.norm(self.F)>self.maxF):
                self.F = self.maxF*self.F/np.linalg.norm(self.F)

           

    def update(self, dt):
        """ 
            Code to update the velocity and position of the agents.  
            as well as determine the new goal velocity 
        """
        if not self.atGoal:
            self.vel += self.F/self.mass *dt     # update the velocity
            self.pos += self.vel*dt   #update the position
        
            # compute the goal velocity for the next time step. Do not modify this
            self.gvel = self.goal - self.pos
            distGoalSq = self.gvel.dot(self.gvel)
            if distGoalSq < self.goalRadiusSq: 
                self.atGoal = True  # goal has been reached
            else: 
                self.gvel = self.gvel/np.sqrt(distGoalSq)*self.prefspeed  


""" 
    Agent class that implements the TTC force-based approach  
"""
class TTCAgent(AbstractAgent):

    def __init__(self, inputParameters, goalRadius=1, dhor = 10, ksi=0.5, timehor=5, epsilon=0, maxF = 10):
        """ 
           Initializes the agent
        """
        super().__init__(inputParameters)
        self.atGoal = False # has the agent reached its goal?
        self.goalRadiusSq = goalRadius * goalRadius # parameter to determine if agent is close to the goal
        self.dhor = dhor # the sensing radius
        self.ksi = ksi # the relaxation time used to compute the goal force
        self.timehor = timehor # the time horizon for computing avoidance forces
        self.epsilon = 0.2 # the error in sensed velocities
        self.F = np.zeros(2) # the total force acting on the agent
        self.maxF = maxF # the maximum force that can be applied to the agent


    def computeAction(self, neighbors=[]):     
        if not self.atGoal:
            # compute goal force
            f_goal = (self.gvel - self.vel)/self.ksi

            # compute repulsive force for each neighbor within sensing radius
            f_avoid = 0
            for n in neighbors:
                
                # remove self when computing neighbors
                if n.id != self.id:
                    d_relative = np.linalg.norm(self.pos - n.pos) - (self.radius + n.radius)
                    # if neighbors are inside sensing radius
                    if d_relative < self.dhor:
                        x_relative = self.pos - n.pos
                        v_relative = self.vel - n.vel
                        ttc = self.computeTTC(n,self.epsilon)
                        if ttc == float('inf'):
                            f_avoid = f_avoid + 0
                        elif ttc == 0:
                            f_avoid = f_avoid + 0
                        else:   
                            n_relative = x_relative + v_relative*ttc / np.linalg.norm(x_relative + v_relative*ttc)
                            f_avoid = f_avoid + (np.max(self.timehor - ttc,0)/ttc) * n_relative
                    
                    # if agents are outside sensing radius
                    else:
                        f_avoid = f_avoid + 0

            # compute final force
            self.F =  f_goal + f_avoid
            if(np.linalg.norm(self.F)>self.maxF):
                self.F = self.maxF*self.F/np.linalg.norm(self.F)
            

    def computeTTC(self,n,eps=0,velSample=False):
        # find Time To Collision (TTC) between agent (self) and neighbor (n)
        x_relative = self.pos - n.pos
        if(velSample):
            v_relative = self.vel - n.vel
        else:
            v_relative = self.vel - n.vel
            
        radius = self.radius + n.radius

        # find discrimant value for TTC
        a = np.dot(v_relative,v_relative) - eps**2
        b = np.dot(x_relative,v_relative) - eps*radius
        c = np.dot(x_relative,x_relative) - radius**2
        # if agents are collision now
        if(c<0):
            return 0

        d = b*b - a*c # discriminant
        # discard imaginary solution (no collision)
        if d<=0:
            return float('inf')

        # if agents are diverding (no collision)
        if b>0:
            return float('inf')

        # compute ttc
        ttc = c/(-b+np.sqrt(d))
        # if collision happened in the past
        if ttc<0:
            return float('inf')
        
        return ttc

    def update(self, dt):
        """ 
            Code to update the velocity and position of the agents.  
            as well as determine the new goal velocity 
        """
        if not self.atGoal:
            self.vel += self.F*dt     # update the velocity
            self.pos += self.vel*dt   #update the position
        
            # compute the goal velocity for the next time step. Do not modify this
            self.gvel = self.goal - self.pos
            distGoalSq = self.gvel.dot(self.gvel)
            if distGoalSq < self.goalRadiusSq: 
                self.atGoal = True  # goal has been reached
            else: 
                self.gvel = self.gvel/np.sqrt(distGoalSq)*self.prefspeed  


""" 
    Agent class that implements a sampling-based VO approach  
"""
class VOAgent(AbstractAgent):

    def __init__(self, inputParameters, goalRadius=1, dhor = 10, epsilon=0):
        """ 
           Initializes the agent
        """
        super().__init__(inputParameters)
        self.atGoal = False # has the agent reached its goal?
        self.goalRadiusSq = goalRadius*goalRadius # parameter to determine if agent is close to the goal
        self.dhor = dhor # the sensing radius
        self.epsilon = epsilon # the error in sensed velocities
        self.vnew = np.zeros(2) # the new velocity of the agent  
   
    def computeAction(self, neighbors=[]):
        """ 
            Your code to compute the new velocity of the agent. 
            The code should set the vnew of the agent to be one of the sampled admissible one. Please do not directly set here the agent's velocity.   
        """       
        
        # sample velocities uniformly from a disc
        n_samples = 200
        v_samples = self.randomSample(self.maxspeed,n_samples)

        # find ttc list of each v_sample
        cost_list = []
        for i in range(n_samples):
            v_candidate = v_samples[:,i]
            ttc_list = []
            for n in neighbors:
                # remove self when computing neighbors
                if n.id != self.id:
                    d_relative = np.linalg.norm(self.pos - n.pos) - (self.radius + n.radius)
                    # find ttc list value for all neighbors
                    # neighbors are inside sensing radius
                    if d_relative < self.dhor:
                        ttc = self.computeTTC(n,self.epsilon,v_candidate,velSample=True)
                        ttc_list.append(ttc)

                    # no neighbours inside sensing radius
                    else:
                        # set ttc_list to inf to handle divide by 0 in cost 3
                        ttc_list.append(float('inf'))

            # find min ttc for all neighbors
            ttc_min = min(ttc_list)
            if ttc_min ==0:
                ttc_min = float('inf')
            # evaluate cost
            cost = self.evaluateCost(v_candidate,ttc_min)
            cost_list.append(cost)

        # find min cost
        idx = np.argmin(cost_list)
        argmin_v = v_samples[:,idx]

        if not self.atGoal:
            self.vnew[:] = argmin_v[:]
            
    def randomSample(self,v_max,n_samples):
        v = v_max * np.sqrt(np.random.rand(n_samples))
        theta = np.random.rand(n_samples) * 2 * np.pi
        v_x = v * np.cos(theta)
        v_y = v * np.sin(theta)
        v_samples = np.array([v_x,v_y])
        return v_samples

    def computeTTC(self,n,eps,v_candidate=[],velSample=False):
        # find Time To Collision (TTC) between agent (self) and neighbor (n)
        x_relative = self.pos - n.pos
        if(velSample):
            v_relative = v_candidate
        else:
            v_relative = self.vel - n.vel
    
        radius = self.radius + n.radius

        # find discrimant value for TTC
        a = np.dot(v_relative,v_relative) - eps**2
        b = np.dot(x_relative,v_relative) - eps*radius
        c = np.dot(x_relative,x_relative) - radius**2
        # if agents are collision now
        if(c<0):
            return 0

        d = b*b - a*c # discriminant
        # discard imaginary solution (no collision)
        if d<=0:
            return float('inf')

        # if agents are diverding (no collision)
        if b>0:
            return float('inf')

        # compute ttc
        ttc = c/(-b+np.sqrt(d))
        # if collision happened in the past
        if ttc<0:
            return float('inf')
        
        return ttc

    def evaluateCost(self,v_candidate,ttc_min,alpha=1, beta=1, gamma=5):
        cost1 = alpha*np.linalg.norm(v_candidate-self.gvel) 
        cost2 = beta*np.linalg.norm(v_candidate-self.vel)
        cost3 = gamma/ttc_min
        return cost1+cost2+cost3

    def update(self, dt):
        """ 
            Code to update the velocity and position of the agent  
            as well as determine the new goal velocity 
        """
        if not self.atGoal:
            self.vel[:] = self.vnew[:]
            self.pos += self.vel*dt   #update the position
        
            # compute the goal velocity for the next time step. Do not modify this
            self.gvel = self.goal - self.pos
            distGoalSq = self.gvel.dot(self.gvel)
            if distGoalSq < self.goalRadiusSq: 
                self.atGoal = True  # goal has been reached
            else: 
                self.gvel = self.gvel/np.sqrt(distGoalSq)*self.prefspeed  