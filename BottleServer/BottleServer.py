from bottle import route, run, get, post, request, static_file, error
import os
import Servo


#NOTE: edit html here - http://www.quackit.com/html/online-html-editor/full/

__version__ = 'Ottershaw Alpha Server Version 0.003'

#-----------------------
#      Setup
#-----------------------

def setRunPath(filepath):
    directoryPath = filepath
    while not directoryPath.endswith('/'):
        directoryPath = directoryPath[:-1]
    directoryPath = directoryPath[:-1]  # remove trailing slash for compatibility with Bottle static routing
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

#TODO implement some of these into a debugger class

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

@get('/addServo')
def addServo():
    return static_file('createservo.html', root = htmlRoot)

@post('/addServo') # or @route('/servo', method='POST')
def doAddServo():
   # TODO -- no access from URL, only via login html files

    '''Create a servo page, adding it to myServos[] array'''
    # TODO - create serialization of created servos for later loading
    print("Adding a servo....")
    servo_ID = int(request.forms.get('servo_ID'))
    bcm_pin = int(request.forms.get('bcm_pin'))
    servo_loc = str(request.forms.get('location'))
    
    # Make sure nobody has entered a servo on this pin already
    if not checkPinUsed(bcm_pin):
        servoToAdd = Servo.servo(servo_loc, bcm_pin, servo_ID)
        myServos.append(servoToAdd)
	    # TODO -- fix broken location in output string
        return "<p> A servo located at " +str(servoToAdd.getLocation) + " was added on pin " + str(servoToAdd.getPin()) + " with a uniqueID of: " + str(servoToAdd.getID()) + "</p>"
    else:
        for servo in myServos:
            if servo.getPin() == bcm_pin:
                return "<p>A servo on this pin already exists with unique ID  = " + servo.getID() + "</p>"
        return "<p>A servo on this pin already exists... but I can't find it. That's bad</p>"

def checkPinUsed(pin):
    '''Check if a pin is already in use by a servo
    Iterate through all created servos checking for 
    duplicate pins
    '''
    for servo in myServos:
        if servo.getPin() == pin:
            return True
    return False

def findServo(ID):
    '''Find servo with a provided unique ID'''
    print("Searcing for servo with provided ID of: " + str(ID))
    for servo in myServos:
        if servo.getID() == int(ID):
            return servo
    return None

@get('/fireServo')
def fireServo():
    return static_file('fireservo.html', root = htmlRoot)

@post('/fireServo')
def doFireServo():
    '''Fire servo page, choosing from myServos[] array'''
   # TODO -- no access from URL, only via login html files

    print("Firing a servo....")
    servo_ID = request.forms.get('servo_ID')
    fire_angle = request.forms.get('fire_angle')
    
    servoToFire = findServo(servo_ID)
    if servoToFire != None:
        servoToFire.rotate(fire_angle)
        return "<p> Fired servo! </p>"
    else:
        return "<p>No servo with this ID could be found. Are you sure you created it? </p>"
   
@get('/saveLoadServos')
def saveLoadServos():
    return static_file('saveLoadServos.html', root = htmlRoot)

@post('/saveLoadServos') # or @route('/servo', method='POST')
def saveLoadServos():
    #Python requires that you explicitly tell it you want to use the global version of myServos
    global myServos

    #Import serialization within Python
    import json       
    
    saveServos = bool(request.forms.get('Save_Servos'))
    loadServos = bool(request.forms.get('Load_Servos'))
    if saveServos == True:
        print("Save servos request")
        # http://www.diveintopython3.net/serializing.html
        
        # Only serialize the servos if they... exist
        if len(myServos) > 0:
            with open('servos.json', mode='w', encoding='utf-8') as f:
                json.dump(myServos, f, indent=2)
            print("Serialized " + str(len(myServos)) + " servos")
    elif loadServos == True:
        print("Load servos request")
        # TODO - CHECK FOR FILE EXIST
        with open('servos.json', 'r', encoding='utf-8') as f:
            myServos = json.load(f)
        print("Loaded " + str(len(myServos)) + " servos")

    # TODO - test JSON serialization. Can create problems in data structures
        # See: http://www.diveintopython3.net/serializing.html

# TODO, with delete functionality
@get('/listServos')
def listServos():
    return static_file('listServos.html', root = htmlRoot)

# TODO: arrow key implementation of servo movement
# Wait for implementation of spider board
@get('/servoController')
def servoController():
    return static_file('servoController.html', root = htmlRoot)


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

# Set all static file serving directories
imageRoot = str(__root) + "/Images"
print("Image root set to " + imageRoot)

htmlRoot = str(__root) + "/html"
print("HTML root set to " + htmlRoot)

currentServoID = 0

# NOTE: CHANGE ME WHEN IMPORTING TO NEW MACHINE
run(host='192.168.1.119', port=8080, debug=True)
#run(host='68.198.140.81', port=8081, debug=True)
