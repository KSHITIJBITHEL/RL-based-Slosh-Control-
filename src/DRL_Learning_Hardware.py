#date : 22 june 2022
# For ARTPARK
#slosh agent for slosh environment 
import numpy as np 
from Linear_SloshDynamics_latest_train   import SloshEnv
import tensorflow as tf
from tensorflow.keras import layers
import matplotlib.pyplot as plt
import random
from tensorflow.keras.models import load_model
from timeit import default_timer as timer
import os
import time 
import serial
ser = serial.Serial('/dev/ttyACM0',baudrate=115200)
ser.flushInput()
os.environ["CUDA_DEVICE_ORDER"] = "PCexirI_BUS_ID" 
os.environ["CUDA_VISIBLE_DEVICES"] = "0"
os.environ['TF_FORCE_GPU_ALLOW_GROWTH'] = 'true' 
flag_episode = 0

class NormalActionNoise:
    def __init__(self, mu, sigma):
        self.mu = mu
        self.sigma = sigma

    def __call__(self):
        return np.random.normal(self.mu, self.sigma)

    def __repr__(self):
        return 'NormalActionNoise(mu={}, sigma={})'.format(self.mu, self.sigma)

# tf.keras.backend.set_floatx('float64')
''' Ornstein-Uhlenbeck Process Action Noise 
    Used by the authors in DDPG paper, it is a time correlated, normally distributed noise 
    but more recent results suggest that uncorrelated, mean-zero Gaussian noise works perfectly well
    May change it if necessary'''
class OUActionNoise:
    def __init__(self, mean, std_deviation, theta=0.8, dt=1e-2, x_initial=None):
        self.theta = theta
        self.mean = mean
        self.std_dev = std_deviation
        self.dt = dt
        self.x_initial = x_initial
        self.reset()
        
    def __call__(self):
        # Formula taken from https://www.wikipedia.org/wiki/Ornstein-Uhlenbeck_process.
        x = (
            self.x_prev
            + self.theta * (self.mean - self.x_prev) * self.dt
            + self.std_dev * np.sqrt(self.dt) * np.random.normal(size=self.mean.shape)
        )
        # Store x into x_prev
        # Makes next noise dependent on current one
        self.x_prev = x
        return x

    def reset(self):
        if self.x_initial is not None:
            self.x_prev = self.x_initial
        else:
            self.x_prev = np.zeros_like(self.mean)




# This update target network parameters slowly
# Based on rate `tau`, which is much less than one.
@tf.function
def update_target(target_weights, weights, tau):
    for (a, b) in zip(target_weights, weights):
        a.assign(b * tau + a * (1 - tau))

''' target networks '''
def get_actor():
    # Initialize weights between -3e-3 and 3-e3
    last_init = tf.random_uniform_initializer(minval=-0.003, maxval=0.003)

    inputs = layers.Input(shape=(num_states,))
    out = layers.Dense(256,dtype = "float32", activation="relu")(inputs)
    out = layers.Dense(256,dtype = "float32",  activation="relu")(out)
    outputs = layers.Dense(num_actions,dtype = "float32",  activation="tanh", kernel_initializer=last_init)(out)

    # Getting outputs in the desired range 
    # mul_matrix = [1,1,1]
    # add_matrix = [0 ,0, 0]
    # mul_matrix[0] = (upper_bound[0]-lower_bound[0])/2
    # mul_matrix[1] = (upper_bound[1]-lower_bound[1])/2
    # mul_matrix[2] = (upper_bound[2]-lower_bound[2])/2
    # add_matrix[0]= (upper_bound[0]+lower_bound[0])/2
    # add_matrix[1]= (upper_bound[1]+lower_bound[1])/2
    # add_matrix[2]= (upper_bound[2]+lower_bound[2])/2
    # outputs = outputs * mul_matrix + add_matrix
    model = tf.keras.Model(inputs, outputs)
    return model


def get_critic():
    # State as input
    state_input = layers.Input(shape=(num_states))
    state_out = layers.Dense(16,dtype = "float32",  activation="relu")(state_input)
    state_out = layers.Dense(32,dtype = "float32",  activation="relu")(state_out)

    # Action as input
    action_input = layers.Input(shape=(num_actions))
    action_out = layers.Dense(32,dtype = "float32",  activation="relu")(action_input)

    # Both are passed through seperate layer before concatenating
    concat = layers.Concatenate()([state_out, action_out])

    out = layers.Dense(256,dtype = "float32",  activation="relu")(concat)
    out = layers.Dense(256,dtype = "float32",  activation="relu")(out)
    outputs = layers.Dense(1, dtype = "float32", )(out)

    # Outputs single value for give state-action
    model = tf.keras.Model([state_input, action_input], outputs)

    return model

"""
`policy()` returns an action sampled from our Actor network plus some noise for
exploration.
"""
class sloshAgent:
     def __init__(self):
        self.std_dev = 0.3
        self.ou_noise1 = NormalActionNoise(mu=np.zeros(1), sigma=0.1 * np.ones(1))#OUActionNoise(mean=np.zeros(1), std_deviation=float(self.std_dev) * np.ones(1))
        # self.ou_noise2 = OUActionNoise(mean=np.zeros(1), std_deviation=float(self.std_dev) * np.ones(1))
        # self.ou_noise3 = OUActionNoise(mean=np.zeros(1), std_deviation=float(self.std_dev) * np.ones(1))
        self.count = 0
        # Use get_actor function to start training and after one iteration use load model function
        self.actor_model =  load_model("actor_main_v2.h5")
        self.critic_model = load_model("critic_main_v2.h5")
        self.reward_data = []
        self.target_actor = load_model("actor_target_v2.h5") #
        self.target_critic = load_model("critic_target_v2.h5") #get_critic() #
        
        # Making the weights equal initially , use for 1st iteration only then comment
        # self.target_actor.set_weights(self.actor_model.get_weights())
        # self.target_critic.set_weights(self.critic_model.get_weights())
        
        # Learning rate for actor-critic models
        self.critic_lr = 0.0015
        self.actor_lr = 0.001
        
        self.critic_optimizer = tf.keras.optimizers.Adam(self.critic_lr)
        self.actor_optimizer = tf.keras.optimizers.Adam(self.actor_lr)
        
        self.total_episodes = 60
        # Discount factor for future rewards
        self.gamma = 0.95
        # Used to update target networks
        self.tau = 0.03 # changed from 0.003
        self.start_time = time.time()
        self.buffer = Buffer(30000,64)

     def run(self):
        global flag_episode
        # To store reward history of each episode
        ep_reward_list = []
        # To store average reward history of last few episodes
        avg_reward_list = []
        
        # Takes about 4 min to train
        for ep in range(self.total_episodes):
        
            prev_state = [0,0,0,0]#env.reset()
            episodic_reward = 0
            flag_episode = 1
            # if ep > total_episodes/2:
            #     std_dev= 0.2
            #     ou_noise = OUActionNoise(mean=np.zeros(1), std_deviation=float(std_dev) * np.ones(1))
            while flag_episode:
                # Uncomment this to see the Actor in action
                # But not in a python notebook.
                
                tf_prev_state = tf.expand_dims(tf.convert_to_tensor(prev_state), 0)
        
                action = self.policy(tf_prev_state, self.ou_noise1,ep)
                # Recieve state and reward from environment.
                # state, reward, done, info = env.step(action)
                # print(prev_state)
                # state,reward,done  = env.simulate(prev_state,7,action,False)
                state,reward  = BEER(action,False)
                print(ep,self.buffer.ret_count(),action,reward,state,round(time.time()-self.start_time,7),'s')
                self.start_time = time.time()
                self.buffer.record((prev_state, action, reward, state))
                episodic_reward += reward
                # if(self.buffer.ret_count()>5): #use 50 for 1st iteration, -1 afterwards
                # self.reward_data.append(reward)
                self.learn()
                self.count += 1
                update_target(self.target_actor.variables, self.actor_model.variables, self.tau)
                update_target(self.target_critic.variables, self.critic_model.variables, self.tau)
                # print("aaya")
                # if (self.count > 10):
                #     self.count = 0 
                #     break
                # # End this episode when `done` is True
                # if done:
                #     self.count = 0 
                #     break
                # break
                # else:
                #     state = [0,0,0,0]#env.reset()
                prev_state = state
            # print("aayaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa")
            ep_reward_list.append(episodic_reward)
        
            # Mean of last 40 episodes
            avg_reward = np.mean(ep_reward_list[-40:])
            # print("Episode * {} * Avg Reward is ==> {}".format(ep, avg_reward))
            # print(timer()-start)
            avg_reward_list.append(avg_reward)
            ep += 1
        reward_plot_x = np.linspace(1,np.size(ep_reward_list),np.size(ep_reward_list))
        plt.plot(reward_plot_x,ep_reward_list)
        plt.show()
        # print(self.reward_data)
        # print(reward_plot_x)
        print("Saving main actor model as actor_main.h5")
        self.actor_model.save("actor_main.h5")
        print("Saving main critic model as critic_main.h5")
        self.critic_model.save("critic_main.h5")
        print("Saving target actor model as actor_target.h5")
        self.target_actor.save("actor_target.h5")
        print("Saving target critic model as critic_target.h5")
        self.target_critic.save("critic_target.h5")
        

     def test(self):
        # self.actor_model= load_model("slosh_actor_N2_v4.h5")
        global flag_episode
        for e in range(2): 
            flag_episode = 1
            cum_reward =0
            state = [0,0,0,0]    #  it will resent initial state for each episode
            while flag_episode:  
               
                tf_state = tf.expand_dims(tf.convert_to_tensor(state), 0)
                sampled_actions = tf.squeeze(self.actor_model(tf_state))
                legal_action = np.clip(sampled_actions, lower_bound, upper_bound)
                legal_action = np.round(legal_action,4)
                action = np.squeeze(legal_action)*upper_bound
                # Recieve state and reward from environment.
                state,reward  = BEER(action,False)
                cum_reward += reward/100
                # print(action)
            print("episode: ", e, "state",state, " action ", action," reward: ", cum_reward)
                
            

     def policy(self,state, noise_object1,ep):
        sampled_actions = tf.squeeze(self.actor_model(state))
        # if(count > 5):
        # if(ep<10):
        #     noise = 0#noise_object1()/10
        # else:
        #     noise = noise_object1()
            # noise2 = noise_object2()
            # noise3 = noise_object3()
            # noise = np.clip([noise1[0]*3,noise2[0]*3,noise3[0]*3],-3,3)
            # print(noise, end =" ")
        # else :
        #     noise = random.uniform(-1,1)#,random.uniform(-3,3),random.uniform(-3,3)]
        noise = 0
        # Adding noise to action
        # print(noise)#, sampled_actions)
        sampled_actions = sampled_actions.numpy() + noise
        #print(sampled_actions)
        #sampled_actions = np.round(sampled_actions,4) 

        # We make sure action is within bounds
        legal_action = sampled_actions*upper_bound
        legal_action = np.round(legal_action,2)
        #print(legal_action)

        return np.squeeze(legal_action)
    
    # Eager execution is turned on by default in TensorFlow 2. Decorating with tf.function allows
    # TensorFlow to build a static graph out of the logic and computations in our function.
    # This provides a large speed up for blocks of code that contain many small TensorFlow operations such as this one.
     @tf.function
     def update(self, state_batch, action_batch, reward_batch, next_state_batch):
        # Training and updating Actor & Critic networks.
        # See Pseudo Code.
        with tf.GradientTape() as tape:
            target_actions = self.target_actor(next_state_batch, training=True)
            y = reward_batch + self.gamma * self.target_critic([next_state_batch, target_actions], training=True)
            critic_value = self.critic_model([state_batch, action_batch], training=True)
            critic_loss = tf.math.reduce_mean(tf.math.square(y - critic_value))

        critic_grad = tape.gradient(critic_loss, self.critic_model.trainable_variables)
        self.critic_optimizer.apply_gradients(
            zip(critic_grad, self.critic_model.trainable_variables)
        )

        with tf.GradientTape() as tape:
            actions = self.actor_model(state_batch, training=True)
            critic_value = self.critic_model([state_batch, actions], training=True)
            # Used `-value` as we want to maximize the value given
            # by the critic for our actions
            actor_loss = -tf.math.reduce_mean(critic_value)

        actor_grad = tape.gradient(actor_loss, self.actor_model.trainable_variables)
        self.actor_optimizer.apply_gradients(
            zip(actor_grad, self.actor_model.trainable_variables)
        )

    # We compute the loss and update parameters
     def learn(self):
        # Get sampling range
        record_range = min(self.buffer.buffer_counter, self.buffer.buffer_capacity)
        # Randomly sample indices
        batch_indices = np.random.choice(record_range, self.buffer.batch_size)

        # Convert to tensors
        state_batch = tf.convert_to_tensor(self.buffer.state_buffer[batch_indices])
        action_batch = tf.convert_to_tensor(self.buffer.action_buffer[batch_indices])
        reward_batch = tf.convert_to_tensor(self.buffer.reward_buffer[batch_indices])
        reward_batch = tf.cast(reward_batch, dtype=tf.float32)
        next_state_batch = tf.convert_to_tensor(self.buffer.next_state_buffer[batch_indices])

        self.update(state_batch, action_batch, reward_batch, next_state_batch)


'''Replay Buffer '''
class Buffer:
    def __init__(self, buffer_capacity=500, batch_size=5):
        # Number of "experiences" to store at max
        self.buffer_capacity = buffer_capacity
        # Num of tuples to train on.
        self.batch_size = batch_size

        # Its tells us num of times record() was called.
        self.buffer_counter = 0

        # Instead of list of tuples as the exp.replay concept go
        # We use different np.arrays for each tuple element
        self.state_buffer = np.zeros((self.buffer_capacity, num_states))
        self.action_buffer = np.zeros((self.buffer_capacity, num_actions))
        self.reward_buffer = np.zeros((self.buffer_capacity, 1))
        self.next_state_buffer = np.zeros((self.buffer_capacity, num_states))

    # Takes (s,a,r,s') obervation tuple as input
    def record(self, obs_tuple):
        # Set index to zero if buffer_capacity is exceeded,
        # replacing old records
        index = self.buffer_counter % self.buffer_capacity

        self.state_buffer[index] = obs_tuple[0]
        self.action_buffer[index] = obs_tuple[1]
        self.reward_buffer[index] = obs_tuple[2]
        self.next_state_buffer[index] = obs_tuple[3]

        self.buffer_counter += 1
    def ret_count(self):
        return self.buffer_counter
''' BEER = Bot by Electrical Engineers of Rajiv bhawan '''
def BEER(action,plot):
    
    ''' Basic Algo :
        send Force value to robot 
        robot takes 1 run with the sent values for 5 sec
        robot sends the accumulated reward and the then state
        bot gets acknowledged for reward data and goes back to the initial position
        Repeat'''
    global flag_episode
    s = str(str(action)+'o')
    ser.write(s.encode())
    decoded_bytes = ""
    # print(s)
    while(decoded_bytes!= "25") :
            ser_bytes = ser.readline()
            decoded_bytes = (ser_bytes[0:len(ser_bytes)-2].decode("utf-8"))
            
            if(decoded_bytes == "##"):
                flag_episode = 0
                print("next episode")
                return [0,0,0,0],0
            if(decoded_bytes!= "25") :
                    # print(decoded_bytes)
                    result = decoded_bytes
    result = result.split(sep=',')
    # print(result[5])
    return [float(result[0]),float(result[1]),float(result[2]),float(result[3])],int(float(result[4]))

if __name__ == "__main__":
    # env = SloshEnv()
    num_states = 4      #{x,xdot,phi,phidot}
    num_actions = 1     #{force}
    upper_bound = 1.2
    lower_bound = -1.2
    
    agent = sloshAgent()
    agent.run()
    agent.test()
    # print(BEER([8,8,4],False))

''' Flow of code 
    * Define agent and actor, critic networks 
    * Initialise target network weights = normal network weights 
    * prev_state = {0,0,0,0}
    * For episodes in total_episodes
        action = actor(prev_state)
        state, reward = env.step(prev_state,action)
        record {prev_state,action,reward,state} in Replay Buffer
        Sample a batch from replay buffer
        Update critic network for minimizing ...
            loss = [reward + critic_target(state,actor_target(state))-critic(prev_state,action)]^2
        Update actor network for maximising ...
            actor_loss = [critic(prev_state,actor(prev_state))] 
        Update target networks 
    '''