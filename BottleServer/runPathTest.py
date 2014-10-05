import os

runPath = os.path.realpath(__file__)
print(runPath)
while not runPath.endswith('/'):
    runPath = runPath[:-1]
print(runPath)

