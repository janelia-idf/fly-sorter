#!/usr/bin/env python
import flask


app = flask.Flask(__name__)

@app.route("/")
def index():
    return flask.render_template('automatic.html')

def webserver():
    # server = 'local'
    server = 'remote'
    port = 5050
    if server == 'local':
        print(' * using debug server - localhost only')
        app.run(debug=True,port=port)
    else:
        print(' * using builtin server - remote access possible')
        app.run(host='0.0.0.0',port=port)


if __name__ == "__main__":
    webserver()
