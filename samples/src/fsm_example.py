# Example of finite state machines
# First we show how to do the example using the pytransitions package
# Make sure the package is installed: pip install transitions; pip install transitions[diagrams]  
#
# In ipython:
#     from fsm_example import MoveBaseRecovery
#     t = MoveBaseRecovery("base")
#     t.state
#     t.stuck() # etc
#     t.get_graph().draw('fsm_example.png', prog='dot')

from transitions.extensions import GraphMachine
import time
import random

class MoveBaseRecovery(object):
  states = ['navigating', 'reset', 'rotate1', 'reset_aggressive', 'rotate2', 'abort']

  def __init__(self, name):
    self.name = name
    self.fsm = GraphMachine(model=self, states=MoveBaseRecovery.states, initial='navigating', show_conditions=True)

  def setup_states(self):
    self.fsm.add_transition(trigger='stuck', source='navigating', dest='reset')
    self.fsm.add_transition(trigger='clear', source='reset', dest='navigating')
    self.fsm.add_transition(trigger='clear', source='rotate1', dest='navigating')
    self.fsm.add_transition(trigger='clear', source='reset_aggressive', dest='navigating')
    self.fsm.add_transition(trigger='clear', source='rotate2', dest='navigating')
    self.fsm.add_transition(trigger='stuck', source='reset', dest='rotate1')
    self.fsm.add_transition(trigger='stuck', source='rotate1', dest='reset_aggressive')
    self.fsm.add_transition(trigger='stuck', source='reset_aggressive', dest='rotate2')
    self.fsm.add_transition(trigger='stuck', source='rotate2', dest='abort')

  def generate_png(self, filename: str):
    self.fsm.get_graph().draw(filename, prog="dot")

  def of_ten(self, odds: int):
    return random.random() < odds/10.0

  def simulate_run(self):
    counter = 0
    while counter < 25 or self.state == "abort":
      counter += 1
      time.sleep(0.5)
      if self.state == "navigating":
        print("navigating")
        if self.of_ten(5):
          print("    oops, I am stuck!")
          self.trigger("stuck")
      elif self.state == "reset":
        print("    resetting and recalculating to try and get unstuck")
        if self.of_ten(5):
          print("    really stuck, trying counter clockwise rotation to try and get unstuck")
          self.trigger("stuck")
        else:
          print("    reset worked!")
          self.trigger("clear")
      elif self.state == "rotate1":
        print("    doing counter clockwise rotatation to try and get unstuck")
        if self.of_ten(5):
          self.trigger("stuck")
        else:
          print("    counter clockwise rotate 1 worked!")
          self.trigger("clear")
      elif self.state == "reset_aggressive":
        print("    doing aggressive reset")
        if self.of_ten(5):
          self.trigger("stuck")
        else:
          print("    aggressive reset worked!\n")
          self.trigger("clear")
      elif self.state == "rotate2":
        print("    doing clockwise rotation to try to get unstuck")
        if self.of_ten(5):
          self.trigger("stuck")
        else:
          print("    rotate clockwise worked!\n")
          self.trigger("clear")

  
example = MoveBaseRecovery("Example")
example.setup_states()
example.generate_png("example.png")
example.simulate_run()