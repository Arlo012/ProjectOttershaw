# This is an example of decorator functions from http://pythonconquerstheuniverse.wordpress.com/2012/04/29/python-decorators/

def originalFunction(input):
  localVar = input
  localVar += 1
  return localVar

def verbosity(originalFunction):

  # add new functioanlity to print out message at start/end of functions
  # kwargs here (confusing still): http://stackoverflow.com/questions/1769403/understanding-kwargs-in-python
  def newFunction(*args, **kwargs):
    print("Entering", originalFunction.__name__)
    originalFunction(*args, **kwargs)
    print("Exiting", originalFunction.__name__)

  return newFunction

def main():
  print(originalFunction(2))
  print(verbosity(originalFunction(3)))


# Run code here
main()


# THIS FUNCTION IS STILL BROKEN
# Also see http://stackoverflow.com/questions/739654/how-can-i-make-a-chain-of-function-decorators-in-python/1594484#1594484