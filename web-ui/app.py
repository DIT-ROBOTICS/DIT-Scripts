from flask import Flask, render_template, request, redirect, url_for
import yaml, json

app = Flask(__name__)

# YAML format
# def load_state():
#     try:
#         with open('/data/state.yaml', 'r') as file:
#             return yaml.safe_load(file)
#     except:
#         return {f'button_{i}': 0 for i in range(6)}
# def save_state(state):
#     with open('/data/state.yaml', 'w') as file:
#         yaml.safe_dump(state, file)

# JSON format
def load_state():
    try:
        with open('/data/state.json', 'r') as file:
            return json.load(file)
    except:
        return {f'button_{i}': 0 for i in range(6)}

# Save the state to a JSON file
def save_state(state):
    with open('/data/state.json', 'w') as file:
        json.dump(state, file)

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

@app.route('/score')
def get_score():
    try:
        with open('/data/score.cache', 'r') as file:
            number = file.read().strip()
    except Exception as e:
        return str(e), 500
    return number

@app.route('/voltage', methods=['GET'])
def get_voltage():
    try:
        with open('/data/voltage.cache', 'r') as file:
            number = file.read().strip()
    except Exception as e:
        return str(e), 500
    return number

@app.route('/group')
def get_group():
    try:
        with open('/data/group.json', 'r') as file:
            data = json.load(file)
    except Exception as e:
        return str(e), 500
    return data

@app.route('/usb')
def get_usb():
    try:
        with open('/data/usb.json', 'r') as file:
            data = json.load(file)
    except Exception as e:
        return str(e), 500
    return data

@app.route('/eurobot2024.appcache')
def cache_manifest():
    response = make_response(open('eurobot2024.appcache').read())
    response.headers['Content-Type'] = 'text/cache-manifest'
    return response

if __name__ == '__main__':
    app.run(host='0.0.0.0', debug=True)

