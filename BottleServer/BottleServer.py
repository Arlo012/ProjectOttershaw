from bottle import route, run, get, post, request, static_file, error
import os
from Servo import servo
from ctypes.wintypes import PINT

#NOTE: edit html here - http://www.quackit.com/html/online-html-editor/full/

__version__ = 'Ottershaw Alpha Server Version 0.002'

#-----------------------
#      Setup
#-----------------------

def setRunPath(filepath):
    directoryPath = filepath
    while not directoryPath.endswith('/'):
        directoryPath = directoryPath[:-1]
    return directoryPath
               
@get('/<filename:re:.*\.(jpg|png|gif|ico)>')
def images(filename):
    '''Static routing of image files'''
    #http://stackoverflow.com/questions/10486224/bottle-static-files
    #http://stackoverflow.com/questions/10480037/static-files-not-being-served-on-bottle-in-python
    return static_file(filename, root= str(imageRoot))


#-----------------------
#   Standard Access
#-----------------------

@route('/')
def home():
    #TODO relative filepath
    #return static_file('home.html', root = '/home/eljefe/Documents/ProjectOttershawCode/BottleServer')
    return static_file('home.html', root = htmlRoot)

@route('/version')
def version():
    return __version__

@route('/theteam')
def theteam():
    return static_file('theteam.html', root = htmlRoot)

@route('/about')
def about():
    return static_file('about.html', root = htmlRoot)


pass
#-----------------------
#   Secure Module
#-----------------------

@get('/login') # or @route('/login')
def login():
    return static_file('login.html', root = htmlRoot)

@post('/login') # or @route('/login', method='POST')
def doLogin():
    username = request.forms.get('username')
    password = request.forms.get('password')
    if checkLogin(username, password):
        #return "<p>Your login information was correct.</p>"
        return admin()
    else:
        return "<p>Login failed.</p>"
    
#Dangerous passwords sitting here. Re-implement later
usernames = ['jeitel', 'jrojas', 'chill', 'bcesar', 'smulliga']
passwords = [ 'toast', 'toast', 'toast', 'toast', 'toast']

def checkLogin(username, password):
    index = 0
    for _user in usernames:
        if _user == username:
            if passwords[index] == password:
                return True
        else:
            index += 1
    
    #No password match
    return False

pass
#-----------------------
# Development & Administration
#-----------------------

def admin():
    return static_file('admin.html', root = htmlRoot)

#TODO - Test me
def restart():
    command = "/usr/bin/sudo /sbin/shutdown -r now"
    import subprocess
    process = subprocess.Popen(command.split(), stdout=subprocess.PIPE)
    output = process.communicate()[0]
    print (output)

myServos = []

@route('/<action>/<item>')
def fireServo(servoID, angle):
    '''Fire locally tracked servo -- create in addServo'''
    myServo = servo("Test Location", 7)
    myServo.rotate(90)

@route('/addServo/')
def addServo(pin):
    return static_file('createservo.html', root = htmlRoot)

@post('/addServo') # or @route('/servo', method='POST')
def doAddServo():
    '''Create a servo page, adding it to myServos[] array'''
    # TODO - create serialization of created servos for later loading
    servo_ID = request.forms.get('servo_ID')
    bcm_pin = request.forms.get('bcm_pin')
    servo_loc = request.forms.get('location')
    
    # Make sure nobody has entered a servo on this pin already
    if checkPinUnique(bcm_pin):
        servoToAdd = servo.servo(servo_loc, bcm_pin, servo_ID)
        myServos.append(servoToAdd)
        return "<p> A servo located at " + servoToAdd.getLocation + " was added on pin " + servoToAdd.getPin() + " with a uniqueID of: " + servoToAdd.getID() + "</p>"
    else:
        for servo in myServos:
            if servo.getPin() == bcm_pin:
                return "<p>A servo on this pin already exists with unique ID = " + servo.getPin() + "</p>"
        return "<p>A servo on this pin already exists... but I can't find it. That's bad</p>"

def checkPinUnique(pin):
    '''Ensure pinout for servo is unique
    Iterate through all created servos checking for 
    duplicate pins
    '''
    for servo in myServos:
        if servo.getPin() == pin:
            return False
    return True

pass
#-----------------------
#     Error Codes
#-----------------------

@error(404)
def error404(error):
    #TODO relative filepath
    return static_file('404.html', root = htmlRoot)

#-----------------------
#     Executed Code
#-----------------------

# Set the working directory of this python instance
__root = setRunPath(os.path.realpath(__file__))

# Set all static serving directories
imageRoot = str(__root) + "/Images"
print("Image root set to " + imageRoot)

htmlRoot = str(__root) + "/html"
print("HTML root set to " + htmlRoot)

currentServoID = 0

# NOTE: CHANGE ME WHEN IMPORTING TO NEW MACHINE
run(host='192.168.1.119', port=8080, debug=True)
#run(host='68.198.140.81', port=8081, debug=True)
