
import numpy as np
import random
#from scipy.fftpack import dct
import json
import sys
import config
from env import make_env
import time
import pybullet

from gym.wrappers import Monitor

final_mode = False
render_mode = True
RENDER_DELAY = False
record_video = False
MEAN_MODE = False

wrapt0 = 0
wrapt1 = 0
wrapt2 = 0
wrapt3 = 0
wrapt4 = 0
mult = 100.

def compress_2d(w, shape=None):
  s = w.shape
  if shape:
    s = shape
  c = dct(dct(w, axis=0, type=2, norm='ortho'), axis=1, type=2, norm='ortho')
  return c[0:s[0], 0:s[1]]

def decompress_2d(c, shape):
  c_out = np.zeros(shape)
  c_out[0:c.shape[0], 0:c.shape[1]] = c
  w = dct(dct(c_out.T, type=3, norm='ortho').T, type=3, norm='ortho')
  return w

def compress_1d(w, shape=None, axis=0):
  s = w.shape
  if shape:
    s = shape
  c = dct(w, axis=axis, type=2, norm='ortho')
  return c[0:s[0], 0:s[1]]

def decompress_1d(c, shape, axis=0):
  c_out = np.zeros(shape)
  c_out[0:c.shape[0], 0:c.shape[1]] = c
  w = dct(c_out, axis=axis, type=3, norm='ortho')
  return w

def make_model(game):
  # can be extended in the future.
  model = Model(game)
  return model

def sigmoid(x):
  return 1 / (1 + np.exp(-x))

def relu(x):
  return np.maximum(x, 0)

def passthru(x):
  return x

def softmax(x):
  e_x = np.exp(x - np.max(x))
  return e_x / e_x.sum(axis=0)

def sample(p):
  return np.argmax(np.random.multinomial(1, p))

class Model:
  ''' simple feedforward model '''
  def __init__(self, game):
    self.output_noise = game.output_noise
    self.env_name = game.env_name
    self.layer_1 = game.layers[0]
    self.layer_2 = game.layers[1]
    self.rnn_mode = False # in the future will be useful
    self.time_input = 0 # use extra sinusoid input
    self.sigma_bias = game.noise_bias # bias in stdev of output
    self.sigma_factor = 0.5 # multiplicative in stdev of output
    if game.time_factor > 0:
      self.time_factor = float(game.time_factor)
      self.time_input = 1
    self.input_size = game.input_size
    self.output_size = game.output_size
    self.shapes = [ (self.input_size + self.time_input, self.layer_1),
                    (self.layer_1, self.layer_2),
                    (self.layer_2, self.output_size)]

    self.sample_output = False
    if game.activation == 'relu':
      self.activations = [relu, relu, passthru]
    elif game.activation == 'sigmoid':
      self.activations = [np.tanh, np.tanh, sigmoid]
    elif game.activation == 'softmax':
      self.activations = [np.tanh, np.tanh, softmax]
      self.sample_output = True
    elif game.activation == 'passthru':
      self.activations = [np.tanh, np.tanh, passthru]
    else:
      self.activations = [np.tanh, np.tanh, np.tanh]

    self.weight = []
    self.bias = []
    self.bias_log_std = []
    self.bias_std = []
    self.param_count = 0

    idx = 0
    for shape in self.shapes:
      self.weight.append(np.zeros(shape=shape))
      self.bias.append(np.zeros(shape=shape[1]))
      self.param_count += (np.product(shape) + shape[1])
      if self.output_noise[idx]:
        self.param_count += shape[1]
      log_std = np.zeros(shape=shape[1])
      self.bias_log_std.append(log_std)
      out_std = np.exp(self.sigma_factor*log_std + self.sigma_bias)
      self.bias_std.append(out_std)
      idx += 1

    self.render_mode = False

  def make_env(self, seed=-1, render_mode=False):
    self.render_mode = render_mode
    self.env = make_env(self.env_name, seed=seed, render_mode=render_mode)

  def get_action(self, x, t=0, mean_mode=False):
    # if mean_mode = True, ignore sampling.
    h = np.array(x).flatten()
    if self.time_input == 1:
      time_signal = float(t) / self.time_factor
      h = np.concatenate([h, [time_signal]])
    num_layers = len(self.weight)
    for i in range(num_layers):
      w = self.weight[i]
      b = self.bias[i]
      h = np.matmul(h, w) + b
      if (self.output_noise[i] and (not mean_mode)):
        out_size = self.shapes[i][1]
        out_std = self.bias_std[i]
        output_noise = np.random.randn(out_size)*out_std
        h += output_noise
      h = self.activations[i](h)

    if self.sample_output:
      h = sample(h)

    return h

  def set_model_params(self, model_params):
    pointer = 0
    for i in range(len(self.shapes)):
      w_shape = self.shapes[i]
      b_shape = self.shapes[i][1]
      s_w = np.product(w_shape)
      s = s_w + b_shape
      chunk = np.array(model_params[pointer:pointer+s])
      self.weight[i] = chunk[:s_w].reshape(w_shape)
      self.bias[i] = chunk[s_w:].reshape(b_shape)
      pointer += s
      if self.output_noise[i]:
        s = b_shape
        self.bias_log_std[i] = np.array(model_params[pointer:pointer+s])
        self.bias_std[i] = np.exp(self.sigma_factor*self.bias_log_std[i] + self.sigma_bias)
        if self.render_mode:
          print("bias_std, layer", i, self.bias_std[i])
        pointer += s

  def load_model(self, filename):
    with open(filename) as f:    
      data = json.load(f)
    print('loading file %s' % (filename))
    self.data = data
    model_params = np.array(data[0]) # assuming other stuff is in data
    self.set_model_params(model_params)

  def get_random_model_params(self, stdev=0.1):
    return np.random.randn(self.param_count)*stdev

def evaluate(model):
  # run 100 times and average score, according to the reles.
  model.env.seed(0)
  total_reward = 0.0
  N = 100
  for i in range(N):
    reward, t = simulate(model, train_mode=False, render_mode=False, num_episode=1)
    total_reward += reward[0]
  return (total_reward / float(N))

def compress_input_dct(obs):
  new_obs = np.zeros((8, 8))
  for i in range(obs.shape[2]):
    new_obs = +compress_2d(obs[:, :, i] / 255., shape=(8, 8))
  new_obs /= float(obs.shape[2])
  return new_obs.flatten()

def simulate(model, train_mode=False, render_mode=True, num_episode=5, seed=-1, max_len=-1):

  reward_list = []
  t_list = []

  orig_mode = True
  dct_compress_mode = False

  max_episode_length = 3000

  if train_mode and max_len > 0:
    if max_len < max_episode_length:
      max_episode_length = max_len

  if (seed >= 0):
    random.seed(seed)
    np.random.seed(seed)
    model.env.seed(seed)

  for episode in range(num_episode):

    if model.rnn_mode:
      model.reset()

    obs = model.env.reset()
    if dct_compress_mode and obs is not None:
      print("compress???")
      obs = compress_input_dct(obs)

    if obs is None:
      obs = np.zeros(model.input_size)

    total_reward = 0.0
    stumbled = False
    reward_threshold = 300 # consider we have won if we got more than this

    for t in range(max_episode_length):
      print("t in max_episode_length:", t)
      if render_mode:
        model.env.render("human")
        if RENDER_DELAY:
          time.sleep(0.01)

      if model.rnn_mode:
        printf("rnn_mode=", model.rnn_mode)
        model.update(obs, t)
        action = model.get_action()
      else:
        print("MEAN_MODE=",MEAN_MODE)
        if MEAN_MODE:
          action = model.get_action(obs, t=t, mean_mode=(not train_mode))
        else:
          action = model.get_action(obs, t=t, mean_mode=False)

      prev_obs = obs

      obs, reward, done, info = model.env.step(action)

      if dct_compress_mode:
        obs = compress_input_dct(obs)

      if train_mode and reward == -100 and (not orig_mode):
        reward = 0
        stumbled = True

      if (render_mode):
        pass
        #print("action", action, "step reward", reward)
        #print("step reward", reward)
      total_reward += reward

      if done:
        if train_mode and (not stumbled) and (total_reward > reward_threshold) and (not orig_mode):
          total_reward += 100
        break

    if render_mode:
      print("reward", total_reward, "timesteps", t)
    reward_list.append(total_reward)
    t_list.append(t)

  return reward_list, t_list




class PendulumDemo():
  def __init__(self):
    gamename = "bullet_pendulum"

    #if gamename.startswith("bullet"):
    #  RENDER_DELAY = True

    use_model = False

    game = config.games[gamename]

  
    use_model = True
    filename = "log/bullet_pendulum.cma.1.28.best.json"
    print("filename", filename)

    the_seed = 0
    if len(sys.argv) > 3:
      the_seed = int(sys.argv[3])
      print("seed", the_seed)

    self.model = make_model(game)
    print('model size', self.model.param_count)

    self.model.make_env(render_mode=render_mode)

    if use_model:
      self.model.load_model(filename)
    else:
      params = model.get_random_model_params(stdev=0.1)
      self.model.set_model_params(params)

  
    self.total_reward = 0.0
    np.random.seed(the_seed)
    self.model.env.seed(the_seed)
    self.reset()
    self.t = 0

  def update(self, context):
    #reward, steps_taken = simulate(self.model, train_mode=False, render_mode=False, num_episode=1)
    
    action = self.model.get_action(self.obs, t=self.t, mean_mode=False)
    
    self.obs, reward, done, info = self.model.env.step(action)
    #self.total_reward += reward[0]
    self.t += 1
    #print("self.t",self.t)
    if (self.t>1000 or done):
    	self.reset()
    
  def reset(self):
    self.t = 0
    self.obs = self.model.env.reset()
    if self.obs is None:
      self.obs = np.zeros(self.model.input_size)
    print ("pendulum reset")



class KukaGraspingDemo():
  def __init__(self):
    gamename = "bullet_kuka_grasping"

    use_model = False

    game = config.games[gamename]

    self.obs = None
    use_model = True
    filename = "zoo/bullet_kuka_grasping_stoc.cma.16.256.best.json";#/bullet_kuka_grasping.cma.16.256.json"
    print("filename", filename)

    the_seed = 0
    if len(sys.argv) > 3:
      the_seed = int(sys.argv[3])
      print("seed", the_seed)

    self.model = make_model(game)
    print('model size', self.model.param_count)

    self.model.make_env(render_mode=render_mode)

    if use_model:
      self.model.load_model(filename)
    else:
      params = model.get_random_model_params(stdev=0.1)
      self.model.set_model_params(params)
    self.total_reward = 0.0
    np.random.seed(the_seed)
    self.model.env.seed(the_seed)
    self.reset()
    self.t = 0

  def update(self, context):
    #reward, steps_taken = simulate(self.model, train_mode=False, render_mode=False, num_episode=1)
    wrapt0 = mult*time.clock()
    action = self.model.get_action(self.obs, t=self.t, mean_mode=False)
    wrapt1 = mult*time.clock()
    self.obs, reward, done, info = self.model.env.step(action)
    #self.obs = np.zeros(self.model.input_size)
    reward = 0
    done = 0
    info = []
    wrapt2 = mult*time.clock()
    
    #print("wrap timing:%.5f,%.5f"%(wrapt1-wrapt0,wrapt2-wrapt1))
    #print("self.obs=",self.obs)
    #self.total_reward += reward[0]
    self.t += 1
    
    if (self.t>1000 or done):
    	self.reset()
    
  def reset(self):
    
    self.t = 0
    self.obs = self.model.env.reset()
    if self.obs is None:
      self.obs = np.zeros(self.model.input_size)
    print ("kuka grasping reset")
