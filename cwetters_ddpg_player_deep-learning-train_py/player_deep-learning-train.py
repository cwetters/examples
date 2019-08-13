#!/usr/bin/python3

# Author(s): Luiz Felipe Vecchietti, Chansol Hong, Inbae Jeong
# Maintainer: Chansol Hong (cshong@rit.kaist.ac.kr)

#Modifier heh...: Christina Wettersten

# Additional Information:
# Train Robot 0 to chase the ball from its coordinates, orientation and the ball coordinates
# GameTime and Deadlock duration can be setup on Webots depending on the number of steps and training details

from __future__ import print_function

from twisted.internet import reactor
from twisted.internet.defer import inlineCallbacks

from autobahn.wamp.serializer import MsgPackSerializer
from autobahn.wamp.types import ComponentConfig
from autobahn.twisted.wamp import ApplicationSession, ApplicationRunner

import argparse
import random
import math
import os
import sys

import base64
import numpy as np
import tensorflow as tf

#from PIL import Image
from dqn_nn import NeuralNetwork
from ddpg import ActorNetwork
from ddpg import CriticNetwork
from ddpg import OrnsteinUhlenbeckActionNoise
from replay_buffer import ReplayBuffer

#reset_reason
NONE = 0
GAME_START = 1
SCORE_MYTEAM = 2
SCORE_OPPONENT = 3
GAME_END = 4
DEADLOCK = 5
GOALKICK = 6
CORNERKICK = 7
PENALTYKICK = 8
HALFTIME = 9
EPISODE_END = 10

episode_enders = {SCORE_MYTEAM, SCORE_OPPONENT, GAME_END, DEADLOCK, HALFTIME, EPISODE_END}
timeout_resets = {GOALKICK, CORNERKICK, PENALTYKICK}

ACTORCHECKPOINT = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'actor/ddpg.actor.ckpt')
CRITICCHECKPOINT = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'critic/ddpg.critic.ckpt')

#game_state
STATE_DEFAULT = 0
STATE_KICKOFF = 1
STATE_GOALKICK = 2
STATE_CORNERKICK = 3
STATE_PENALTYKICK = 4

#coordinates
MY_TEAM = 0
OP_TEAM = 1
BALL = 2
X = 0
Y = 1
TH = 2
ACTIVE = 3
TOUCH = 4

#path to your checkpoint
CHECKPOINT = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'dqn.ckpt')

class Received_Image(object):
    def __init__(self, resolution, colorChannels):
        self.resolution = resolution
        self.colorChannels = colorChannels
        # need to initialize the matrix at timestep 0
        self.ImageBuffer = np.zeros((resolution[1], resolution[0], colorChannels)) # rows, columns, colorchannels
    def update_image(self, received_parts):
        self.received_parts = received_parts
        for i in range(0,len(received_parts)):
           dec_msg = base64.b64decode(self.received_parts[i].b64, '-_') # decode the base64 message
           np_msg = np.fromstring(dec_msg, dtype=np.uint8) # convert byte array to numpy array
           reshaped_msg = np_msg.reshape((self.received_parts[i].height, self.received_parts[i].width, 3))
           for j in range(0, self.received_parts[i].height): # y axis
               for k in range(0, self.received_parts[i].width): # x axis
                   self.ImageBuffer[j+self.received_parts[i].y, k+self.received_parts[i].x, 0] = reshaped_msg[j, k, 0] # blue channel
                   self.ImageBuffer[j+self.received_parts[i].y, k+self.received_parts[i].x, 1] = reshaped_msg[j, k, 1] # green channel
                   self.ImageBuffer[j+self.received_parts[i].y, k+self.received_parts[i].x, 2] = reshaped_msg[j, k, 2] # red channel

class SubImage(object):
    def __init__(self, x, y, width, height, b64):
        self.x = x
        self.y = y
        self.width = width
        self.height = height
        self.b64 = b64

class Frame(object):
    def __init__(self):
        self.time = None
        self.score = None
        self.reset_reason = None
        self.subimages = None
        self.coordinates = None
        self.half_passed = None


# ===========================
#   Tensorflow Summary Ops
# ===========================

def build_summaries():
    episode_reward = tf.Variable(0.)
    tf.summary.scalar("Reward", episode_reward)
    episode_ave_max_q = tf.Variable(0.)
    tf.summary.scalar("Qmax Value", episode_ave_max_q)

    summary_vars = [episode_reward, episode_ave_max_q]
    summary_ops = tf.summary.merge_all()

    return summary_ops, summary_vars


class Component(ApplicationSession):
    """
    AI Base + ddpg port from Ptrick Emami's implementation
    """

    def __init__(self, config, sess):
        ApplicationSession.__init__(self, config)
        self.sess = sess

    def printConsole(self, message):
        print(message)
        sys.__stdout__.flush()

    def onConnect(self):
        self.join(self.config.realm)

    @inlineCallbacks
    def onJoin(self, details):

##############################################################################
        def init_variables(self, info):
            # Here you have the information of the game (virtual init() in random_walk.cpp)
            # List: game_time, number_of_robots
            #       field, goal, penalty_area, goal_area, resolution Dimension: [x, y]
            #       ball_radius, ball_mass,
            #       robot_size, robot_height, axle_length, robot_body_mass, ID: [0, 1, 2, 3, 4]
            #       wheel_radius, wheel_mass, ID: [0, 1, 2, 3, 4]
            #       max_linear_velocity, max_torque, codewords, ID: [0, 1, 2, 3, 4]
            # self.game_time = info['game_time']
            # self.number_of_robots = info['number_of_robots']

            # self.field = info['field']
            # self.goal = info['goal']
            # self.penalty_area = info['penalty_area']
            # self.goal_area = info['goal_area']
            self.resolution = info['resolution']

            # self.ball_radius = info['ball_radius']
            # self.ball_mass = info['ball_mass']

            # self.robot_size = info['robot_size']
            # self.robot_height = info['robot_height']
            # self.axle_length = info['axle_length']
            # self.robot_body_mass = info['robot_body_mass']

            # self.wheel_radius = info['wheel_radius']
            # self.wheel_mass = info['wheel_mass']

            self.max_linear_velocity = info['max_linear_velocity']
            # self.max_torque = info['max_torque']
            # self.codewords = info['codewords']

            self.colorChannels = 3 # nf
            self.end_of_frame = False
            self.image = Received_Image(self.resolution, self.colorChannels)
            self._frame = 0
            

            ## ddpg ports
            self.actor_lr = 0.0001
            self.critic_lr = 0.001
            self.tau = 0.001
            self.buffersize = 1000000
            self.minibatch_size = 64
            self.gamma = 0.99
            self.max_ep = 50000
            self.max_ep_len = 1000
            self.max_no_touch = 100
            self.action_dim = 10
            self.state_dim = 32
            self.action_bound = 1.0
            self.summary_dir = './results/tf_ddpg'
            self.resetting = False
            self.timedout = False
            self.ep_reward = 0
            self.ep_ave_max_q = 0
            self.lastaction = np.zeros(self.action_dim)
            self.laststate = np.zeros(self.state_dim)

            self.actor = ActorNetwork(self.sess, self.state_dim, self.action_dim, self.action_bound, 
                                float(self.actor_lr), float(self.tau),
                                int(self.minibatch_size), ACTORCHECKPOINT) #last false to ACTORCHECKPOINT to load file
            
            self.critic = CriticNetwork(self.sess, self.state_dim, self.action_dim,
                                float(self.critic_lr), float(self.tau),
                                float(self.gamma),
                                self.actor.get_num_trainable_vars(), CRITICCHECKPOINT) #last false to CRITICCHECKPOINT to load file
            
            self.actor_noise = OrnsteinUhlenbeckActionNoise(mu=np.zeros(self.action_dim))

            self.summary_ops, self.summary_vars = build_summaries()

            self.sess.run(tf.global_variables_initializer())
            self.writer = tf.summary.FileWriter(self.summary_dir, self.sess.graph)

            self.actor.update_target_network()
            self.critic.update_target_network()

            self.replay_buffer = ReplayBuffer(int(self.buffersize))

            self.iter = 0
            self.ep = 0
            self.touchcount = 0




            self.wheels = [0 for _ in range(10)]
            return
##############################################################################

        try:
            info = yield self.call(u'aiwc.get_info', args.key)
        except Exception as e:
            self.printConsole("Error: {}".format(e))
        else:
            try:
                self.sub = yield self.subscribe(self.on_event, args.key)
            except Exception as e2:
                self.printConsole("Error: {}".format(e2))

        init_variables(self, info)

        try:
            yield self.call(u'aiwc.ready', args.key)
        except Exception as e:
            self.printConsole("Error: {}".format(e))
        else:
            self.printConsole("I am ready for the game!")

    @inlineCallbacks
    def on_event(self, f):

        @inlineCallbacks
        def set_wheel(self, robot_wheels):
            yield self.call(u'aiwc.set_speed', args.key, robot_wheels)
            return

        def set_action(a):
            #self.printConsole(a)
            for i in range (self.action_dim):
                playerid = int(i/2)
                maxspeed = self.max_linear_velocity[playerid]
                self.wheels[i] = a[0][i]*maxspeed
            
           
        def distance(x1, x2, y1, y2):
            return math.sqrt(math.pow(x1 - x2, 2) + math.pow(y1 - y2, 2))

        # initiate empty frame
        received_frame = Frame()
        received_subimages = []

        if 'time' in f:
            received_frame.time = f['time']
        if 'score' in f:
            received_frame.score = f['score']
        if 'reset_reason' in f:
            received_frame.reset_reason = f['reset_reason']
        if 'half_passed' in f:
            received_frame.half_passed = f['half_passed']
        if 'subimages' in f:
            received_frame.subimages = f['subimages']
            # Comment the next lines if you don't need to use the image information
            for s in received_frame.subimages:
                received_subimages.append(SubImage(s['x'],
                                                   s['y'],
                                                   s['w'],
                                                   s['h'],
                                                   s['base64'].encode('utf8')))
            self.image.update_image(received_subimages)
        if 'coordinates' in f:
            received_frame.coordinates = f['coordinates']
        if 'EOF' in f:
            self.end_of_frame = f['EOF']

        #self.printConsole(received_frame.time)
        #self.printConsole(received_frame.score)
        #self.printConsole(received_frame.reset_reason)
        #self.printConsole(self.end_of_frame)

        if (self.end_of_frame):
            self._frame += 1

            # To get the image at the end of each frame use the variable:
            #self.printConsole(self.image.ImageBuffer)

##############################################################################
            #(virtual update())

            # Reward 
            
            # State [robot id, robot x, robot y, robot theta, ball x, ball y]

            # If you want to use the image as the input for your network
            # You can use pillow: PIL.Image to get and resize the input frame as follows
            #img = Image.fromarray((self.image.ImageBuffer/255).astype('uint8'), 'RGB') # Get normalized image as a PIL.Image object
            #resized_img = img.resize((NEW_X,NEW_Y))
            #final_img = np.array(resized_img)


            bx = received_frame.coordinates[BALL][X]
            by = received_frame.coordinates[BALL][Y]

            r=0
            touchinc = 0
            position = []
            for i in range(5):
                mx = received_frame.coordinates[MY_TEAM][i][X]
                my = received_frame.coordinates[MY_TEAM][i][Y]
                mt = received_frame.coordinates[MY_TEAM][i][TH]
                ox = received_frame.coordinates[OP_TEAM][i][X]
                oy = received_frame.coordinates[OP_TEAM][i][Y]
                ot = received_frame.coordinates[OP_TEAM][i][TH]
                if(received_frame.coordinates[MY_TEAM][i][TOUCH]):
                    self.touchcount = 0
                    r += 1
                elif(received_frame.coordinates[OP_TEAM][i][TOUCH]):
                    touchinc = 5
                    
                else:
                    touchinc = 1

                
                position.extend([round(mx/3.9, 2), round(my/2.45, 2),
                            round(mt/(2*math.pi), 2), round(ox/3.9, 2), round(oy/2.45, 2),
                            round(ot/(2*math.pi), 2)])

            position.extend([bx, by])
            if not (self.resetting or self.timedout):
                action = self.actor.predict(np.reshape(position, (1, self.actor.s_dim))) #+ self.actor_noise()
            else:
                action = np.array([np.zeros(self.action_dim)]) #if we're waiting for reset don't move to get to reset faster
            if (self.laststate == np.zeros(self.state_dim)).all():
                self.laststate = position
            
            terminal = False
            reset = False
            endtimeout = False
            
            if received_frame.reset_reason in episode_enders:
                if received_frame.reset_reason == SCORE_MYTEAM:
                        if(self.touchcount < self.max_no_touch):
                            r += 100
                        else:
                            r+= 10
                elif received_frame.reset_reason == SCORE_OPPONENT:
                        if(self.touchcount < self.max_no_touch):
                            r += -100
                        else:
                            r+= -10
                terminal = True
                reset = True
            elif received_frame.reset_reason in timeout_resets:
                endtimeout = True

            if not (self.resetting or self.timedout):
                self.touchcount += touchinc
                self.replay_buffer.add(np.reshape(self.laststate, (self.actor.s_dim,)), np.reshape(self.lastaction, (self.actor.a_dim,)), r,
                                    terminal, np.reshape(position, (self.actor.s_dim,)))
                
                ##############################################################################

                ##############################################################################
                if(self.ep < self.max_ep):
                    if(self.iter < self.max_ep_len ):#and self.touchcount <= self.max_no_touch):
                        # Training!
                        #self.printConsole(len(self.D[0]))
                        if self.replay_buffer.size() > self.minibatch_size:
                            s_batch, a_batch, r_batch, t_batch, s2_batch = self.replay_buffer.sample_batch(self.minibatch_size)

                            target_q = self.critic.predict_target(s2_batch, self.actor.predict_target(s2_batch))

                            y_i = []
                            for k in range(self.minibatch_size):
                                if t_batch[k]:
                                    y_i.append(r_batch[k])
                                else:
                                    y_i.append(r_batch[k] + self.critic.gamma * target_q[k])
                            
                            predicted_q_value, _ = self.critic.train(s_batch, a_batch, np.reshape(y_i, (self.minibatch_size, 1)))

                            self.ep_ave_max_q += np.amax(predicted_q_value)

                            a_outs = self.actor.predict(s_batch)
                            grads = self.critic.action_gradients(s_batch, a_outs)
                            self.actor.train(s_batch, grads[0])

                            self.actor.update_target_network()
                            self.critic.update_target_network()
                        self.iter += 1
                        self.ep_reward += r

                        if terminal:
                            summary_str = sess.run(self.summary_ops, feed_dict={
                                self.summary_vars[0]: self.ep_reward,
                                self.summary_vars[1]: self.ep_ave_max_q/float(self.iter)
                            })

                            self.writer.add_summary(summary_str, self.ep)
                            self.writer.flush()

                            self.printConsole('terminal | Reward: {:d} | Episode: {:d} | Qmax: {:4f}'.format(int(self.ep_reward), self.ep, (self.ep_ave_max_q / float (self.iter))))
                            if self.ep %10 == 9:
                                self.actor.SaveToFile(ACTORCHECKPOINT)
                                self.critic.SaveToFile(CRITICCHECKPOINT)
                            self.ep_reward = 0
                            self.ep_ave_max_q = 0
                            self.ep += 1
                            self.iter = 0
                            self.actor_noise.reset()
                            self.touchcount = 0
                    else:
                        summary_str = sess.run(self.summary_ops, feed_dict={
                                self.summary_vars[0]: self.ep_reward,
                                self.summary_vars[1]: self.ep_ave_max_q/float(self.iter)
                            })

                        self.writer.add_summary(summary_str, self.ep)
                        self.writer.flush()
 
                        self.printConsole('timedout | Reward: {:d} | Episode: {:d} | Qmax: {:4f}'.format(int(self.ep_reward), self.ep, (self.ep_ave_max_q / float (self.iter))))
                        if self.ep % 10 == 9:
                            self.actor.SaveToFile(ACTORCHECKPOINT)
                            self.critic.SaveToFile(CRITICCHECKPOINT)
                        self.timedout = True
                        self.ep_reward = 0
                        self.ep_ave_max_q = 0
                        self.ep += 1
                        self.iter = 0
                        self.actor_noise.reset()
                        self.touchcount = 0


            # Set robot wheels
            set_action(action)
            set_wheel(self, self.wheels)

            

            self.resetting = reset
            if self.resetting:
                self.timedout = False
            if endtimeout:
                self.timedout = False
            
            self.laststate = position
            self.lastaction = action          

##############################################################################

            if(received_frame.reset_reason == GAME_END):

##############################################################################
                #(virtual finish() in random_walk.cpp)
                #save your data
                with open(args.datapath + '/result.txt', 'w') as output:
                    #output.write('yourvariables')
                    output.close()
                #unsubscribe; reset or leave
                yield self.sub.unsubscribe()
                try:
                    yield self.leave()
                except Exception as e:
                    self.printConsole("Error: {}".format(e))
##############################################################################

            self.end_of_frame = False


    def onDisconnect(self):
        if reactor.running:
            reactor.stop()

if __name__ == '__main__':

    try:
        unicode
    except NameError:
        # Define 'unicode' for Python 3
        def unicode(s, *_):
            return s

    def to_unicode(s):
        return unicode(s, "utf-8")

    parser = argparse.ArgumentParser()
    parser.add_argument("server_ip", type=to_unicode)
    parser.add_argument("port", type=to_unicode)
    parser.add_argument("realm", type=to_unicode)
    parser.add_argument("key", type=to_unicode)
    parser.add_argument("datapath", type=to_unicode)

    # agent parameters
    parser.add_argument('--actor-lr', help='actor network learning rate', default=0.0001)
    parser.add_argument('--critic-lr', help='critic network learning rate', default=0.001)
    parser.add_argument('--gamma', help='discount factor for critic updates', default=0.99)
    parser.add_argument('--tau', help='soft target update parameter', default=0.001)
    parser.add_argument('--buffer-size', help='max size of the replay buffer', default=1000000)
    parser.add_argument('--minibatch-size', help='size of minibatch for minibatch-SGD', default=64)

    # run parameters
    parser.add_argument('--env', help='choose the gym env- tested on {Pendulum-v0}', default='Pendulum-v0')
    parser.add_argument('--random-seed', help='random seed for repeatability', default=1234)
    parser.add_argument('--max-episodes', help='max num of episodes to do while training', default=50000)
    parser.add_argument('--max-episode-len', help='max length of 1 episode', default=1000)
    parser.add_argument('--render-env', help='render the gym env', action='store_true')
    parser.add_argument('--use-gym-monitor', help='record gym results', action='store_true')
    parser.add_argument('--monitor-dir', help='directory for storing gym results', default='./results/gym_ddpg')
    parser.add_argument('--summary-dir', help='directory for storing tensorboard info', default='./results/tf_ddpg')


    args = parser.parse_args()

    ai_sv = "rs://" + args.server_ip + ":" + args.port
    ai_realm = args.realm

    # create a Wamp session object
    with tf.Session() as sess:
        session = Component(ComponentConfig(ai_realm, {}), sess)

        # initialize the msgpack serializer
        serializer = MsgPackSerializer()

        # use Wamp-over-rawsocket
        runner = ApplicationRunner(ai_sv, ai_realm, serializers=[serializer])

        runner.run(session, auto_reconnect=False)
