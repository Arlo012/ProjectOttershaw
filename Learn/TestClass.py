#------------------------------------
# Learning how Python classes work
# http://www.jesshamrick.com/2011/05/18/an-introduction-to-classes-and-inheritance-in-python/
#------------------------------------

class Pet(object):
  '''This is a docstring'''
  def __init__(self, name, species):
    self.name = name
    self.species = species

  def getName(self):
    return self.name

  def getSpecies(self):
    return self.species

  def __str__(self):
    return "%s is a %s" % (self.name, self.species)



# Stupid contrived example of a Dog inheriting from the pet class
class Dog(Pet):
  '''I am a dog. Woof woof mother fucker'''
  def __init__(self, name, chases_cats):
    Pet.__init__(self, name, "Dog")
    self.chases_cats = chases_cats


  def chasesCats(self):
    return self.chases_cats

