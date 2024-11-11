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
    while True:
      if self.fsm.state() == "navigating":
        print("navigating")
        if self.of_ten(1):
          print("oops, I am stuck!")
          self.fsm.trigger("stuck")
      elif self.fsm.state() == "reset":
        print("resetting...")
        if self.of_ten(1):
          print("really stuck, trying rotate")
          self.fsm.trigger("clear")
      else:
        print("bug in Pito's code")
        break

  
example = MoveBaseRecovery("Example")
example.setup_states()
example.generate_png("example.png")
example.simulate_run()