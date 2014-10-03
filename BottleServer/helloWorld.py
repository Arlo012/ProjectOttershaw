from bottle import route, run


# Note: this @ symbol is a decorator
# See good explanation HERE: http://pythonconquerstheuniverse.wordpress.com/2012/04/29/python-decorators/
@route('/hello')
def hello():
    return "Hello World!"

run(host='localhost', port=8080, debug=True)