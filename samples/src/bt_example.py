import py_trees
import random
import time


class Prompted(py_trees.Behaviour):
  def __init__(self, name):
    self.prompt = "\n***> " + name + " (running, success, failure) -- "
    super(Prompted, self).__init__(name)
  
  def update(self):
    response = input(self.prompt)
    while not response.lower() in ["running", "failure", "success", "r", "f", "s"]:
      print ("Please enter running, success or failure ...")
      response = input(self.prompt)
    if response.lower() in ["running", "r"]:
      print ("Behavior: " + self.name +  " is running")
      return py_trees.Status.RUNNING
    elif response.lower() in ["failure", "f"]:
      print ("Behavior: " + self.name +  " has failed")
      return py_trees.Status.FAILURE
    elif response.lower() in ["success", "s"]:
      print ("Behavior: " + self.name +  " has succeeeded")
      return py_trees.Status.SUCCESS
    else:
      print("ERROR - NO MATCH")

def create_tree():
  recover = py_trees.composites.Selector("Recover")
  cons_reset = Prompted("Conservative Reset")
  rotate = Prompted("Clearing Rotation")
  aggressive = Prompted("Aggressive Reset")
  recover.add_child(cons_reset)
  recover.add_child(rotate)
  recover.add_child(aggressive)
  recover.add_child(rotate)
  root = py_trees.composites.Selector("move_base")
  navigate = Prompted("navigate")
  root.add_child(navigate)
  root.add_child(recover)
  return root

if __name__ == "__main__":
  tr = create_tree()
  py_trees.display.render_dot_tree(tr)
  try:
    for _ in range(0,100):
      tr.tick_once()
      # py_trees.display.print_ascii_tree(tr, show_status=True)
      if (tr.status == py_trees.Status.SUCCESS):
        print("\n***> Navigation complete!")
        exit(0)
      elif (tr.status == py_trees.Status.FAILURE):
        print("\n***> Navigation FAILED!!")
        exit(0)
      time.sleep(0.25)
  except KeyboardInterrupt:
    pass

