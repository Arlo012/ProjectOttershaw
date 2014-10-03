from bottle import route, run, get, post, request, static_file, error
import os


#NOTE: edit html here - http://www.quackit.com/html/online-html-editor/full/

__version__ = 'Ottershaw Alpha Server Version 0.001'

#-----------------------
#      Setup
#-----------------------

@get('/<filename:re:.*\.(jpg|png|gif|ico)>')
def images(filename):
    #TODO relative filepath
    #http://stackoverflow.com/questions/10486224/bottle-static-files
    #http://stackoverflow.com/questions/10480037/static-files-not-being-served-on-bottle-in-python
    return static_file(filename, root='/home/eljefe/Documents/ProjectOttershawCode/BottleServer/Images')

pass
#-----------------------
#   Standard Access
#-----------------------

@route('/')
def home():
    #TODO relative filepath
    return static_file('home.html', root = '/home/eljefe/Documents/ProjectOttershawCode/BottleServer')

@route('/version')
def version():
    return __version__

@route('/theteam')
def theteam():
    return static_file('theteam.html', root = '/home/eljefe/Documents/ProjectOttershawCode/BottleServer')

@route('/about')
def about():
    return static_file('about.html', root = '/home/eljefe/Documents/ProjectOttershawCode/BottleServer')


pass
#-----------------------
#   Secure Module
#-----------------------

@get('/login') # or @route('/login')
def login():
    return static_file('login.html', root = '/home/eljefe/Documents/ProjectOttershawCode/BottleServer')

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
    index = 1
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
    return static_file('admin.html', root = '/home/eljefe/Documents/ProjectOttershawCode/BottleServer')

def restart():
    command = "/usr/bin/sudo /sbin/shutdown -r now"
    import subprocess
    process = subprocess.Popen(command.split(), stdout=subprocess.PIPE)
    output = process.communicate()[0]
    print (output)

pass
#-----------------------
#     Error Codes
#-----------------------

@error(404)
def error404(error):
    #TODO relative filepath
    return static_file('404.html', root = '/home/eljefe/Documents/ProjectOttershawCode/BottleServer')


run(host='192.168.1.147', port=8080, debug=True)