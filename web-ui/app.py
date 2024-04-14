from flask import Flask, render_template, request, redirect, url_for
import yaml

app = Flask(__name__)


def load_state():
    try:
        with open('/data/state.yaml', 'r') as file:
            return yaml.safe_load(file)
    except:
        return {f'button_{i}': 0 for i in range(6)}

def save_state(state):
    with open('/data/state.yaml', 'w') as file:
        yaml.safe_dump(state, file)


@app.route('/', methods=['GET', 'POST'])
def index():
    if request.method == 'POST':
        state = load_state()
        button_id = request.form['button']
        state[button_id] = 1 - state[button_id]  # Toggle the state
        save_state(state)
        return redirect(url_for('index'))
    else:
        state = load_state()
        return render_template('index.html', state=state)

@app.route('/eurobot2024.appcache')
def cache_manifest():
    response = make_response(open('eurobot2024.appcache').read())
    response.headers['Content-Type'] = 'text/cache-manifest'
    return response

if __name__ == '__main__':
    app.run(host='0.0.0.0', debug=True)

