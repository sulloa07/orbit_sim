import sys, math

# globals
position = [0, 0] # [x, y] meters
Vvec = [0, 0] # [vx, vy] m/s
time_elapsed = 0 # seconds
trajectory = [] # flight path points
flight = {'events': []} # flight definition

# rocket defaults
rocket = {
    'mass': 1.0, 'fuel':0.5, 'thrust':20.0, 'burnRate': 0.1, 'diameter':0.03, 'Cd': 1.14,
    'fizzbuzz': False  # added fizzbuzz functionality (default off)
}
# environment defaults
env = {'gravity': 9.81, 'drag': 0.1}

# main simulation function
# contains all the physics and logic for the simulation
# this runs through and applies the events to the rocket durign flight and sdjusts accordingly
# it also handles the trajectory and the display of the trajectory
# currently only handles ballistic flight, no guidance or control or parachutes
def run_simulation(sim):
    global position, Vvec, time_elapsed, trajectory
    print("\nRunning simulation")
    print("-" * 40)
    # Rocket settings
    mass = rocket['mass'] # initial mass in kg
    fuel = rocket['fuel'] # initial fuel in kg
    thrust = rocket['thrust'] # thrust in N
    burn_rate = rocket['burnRate'] # fuel burn rate in kg/s
    area = rocket['area'] # cross-sectional area in m^2
    Cd = rocket['Cd'] # drag coefficient
    fizzbuzz_enabled = rocket['fizzbuzz']  # FizzBuzz flag
    
    # Simulation settings (defaults)
    thrust_angle = 90  # default vertical
    power = 0 # default engine off
    gravity = env['gravity']
    dt = 0.1 # time step
    
    # sort and extract events
    time_events = sorted([e for e in flight['events'] if e['condition_type'] == 't'], 
                         key=lambda e: e['condition_value'])
    altitude_events = [e for e in flight['events'] if e['condition_type'] == 'altitude']
    event_index = 0
    
    # apply t=0 events
    for event in time_events:
        if event['condition_type'] == 't' and event['condition_value'] == 0:
            for action in event['actions']: # apply actions statement 
                if action['type'] == 'set':
                    if action['property'] == 'angle': 
                        thrust_angle = action['value']
                        print(f"t=0.0s: angle={thrust_angle}°")
                    elif action['property'] == 'power': 
                        power = action['value']
                        print(f"t=0.0s: power={power}%")
                    elif action['property'] == 'mass': 
                        mass = action['value']
                        print(f"t=0.0s: mass={mass}kg")
    
    # Main simulation loop
    # max_time =  3600  # max 1 hr sim time
    simulation_complete = False
    
    while not simulation_complete: # and time_elapsed < max_time:
        trajectory.append((position[0], position[1])) # update trajectory
        # print(round(time_elapsed))
        # FizzBuzz check - every 3 seconds or 5 seconds or both
        if fizzbuzz_enabled and time_elapsed > 0:
            int_time = int(round(time_elapsed))
            if int_time % 3 == 0 and int_time % 5 == 0:
                print(f"At {time_elapsed:.1f}s altitude={position[1]:.1f}m: Deployed FIZZBUZZ!")
            elif int_time % 3 == 0:
                print(f"At {time_elapsed:.1f}s altitude={position[1]:.1f}m: Deployed FIZZ!")
            elif int_time % 5 == 0:
                print(f"At {time_elapsed:.1f}s altitude={position[1]:.1f}m: Deployed BUZZ!")
                    
        # process time events
        while (event_index < len(time_events) and time_events[event_index]['condition_value'] <= time_elapsed): 
            for action in time_events[event_index]['actions']: # apply time actions to simulation
                if action['type'] == 'set':
                    if action['property'] == 'angle':
                        thrust_angle = action['value']
                        print(f"t={time_elapsed:.1f}s: angle={thrust_angle}°")
                    elif action['property'] == 'power':
                        power = action['value']
                        print(f"t={time_elapsed:.1f}s: power={power}%")
                    elif action['property'] == 'mass':
                        mass = action['value']
                        print(f"t={time_elapsed:.1f}s: mass={mass}kg")
            event_index += 1
        
        # check altitude events
        for event in altitude_events:
            for action in event['actions']: # apply altitude actions to simulation
                if action['type'] == 'set': 
                    if action['property'] == 'angle':
                        thrust_angle = action['value']
                        print(f"alt={position[1]:.1f}m: angle={thrust_angle}°")
                    elif action['property'] == 'power':
                        power = action['value']
                        print(f"alt={position[1]:.1f}m: power={power}%")
                    elif action['property'] == 'mass':
                        mass = action['value']
                        print(f"alt={position[1]:.1f}m: mass={mass}kg")
                event.processed = True
        
        # find rocket forces 
        # reference: https://rocketmime.com/rockets/rckt_sim.html
        
        # THRUST FORCE 
        # apply when engine is on and fuel is available
        if fuel > 0:
            Ttotal = thrust * (power / 100)
        else:
            Ttotal = 0.0
        # seperate thrust into x and y components using thrust angle
        Tx = Ttotal * math.cos(math.radians(thrust_angle))
        Ty = Ttotal * math.sin(math.radians(thrust_angle))
        
        # GRAVITY
        # gravity force is always acting downwards
        # F = m * g
        Fg = mass * gravity  # This is always a negative force in y direction
        
        # DRAG FORCE
        # always acting opposite to the direction of motion
        # Fd = 0.5 * rho * Cd * A * v^2
        current_rho = calculate_air_density(position[1]) 
        Vtotal = math.sqrt(Vvec[0]**2 + Vvec[1]**2)
        if Vtotal > 0: # only apply drag if there is motion
            Fd = 0.5 * current_rho * Cd * area * Vtotal**2
            Fdx = -Fd * (Vvec[0] / Vtotal)
            Fdy = -Fd * (Vvec[1] / Vtotal)
        else:
            Fdx = Fdy = 0
        # TOTAL FORCES
        # F = Ft - Fd - M*g (for y-direction)
        Fx = Tx + Fdx
        Fy = Ty - Fg + Fdy  # gravity is subtracted
        # ACCELERATION UPDATE
        # a = F/m
        ax = Fx / mass
        ay = Fy / mass
        # VELOCiTY UPDATE
        # v = v0 + at
        Vvec[0] += ax * dt
        Vvec[1] += ay * dt
        # POSITION UPDATE
        #  x = x0 + vt
        position[0] += Vvec[0] * dt
        position[1] += Vvec[1] * dt
        # did we crash?
        if position[1] < 0:
            position[1] = 0
            print(f"Rocket landed at x={position[0]:.1f}m after {time_elapsed:.1f}s")
            simulation_complete = True # finish dat shi
            break
        # print(f"t={time_elapsed:.1f} thrust={thrust_force:.1f} accel=({accel_x:.2f},{accel_y:.2f}) vel=({velocity[0]:.2f},{velocity[1]:.2f}) pos=({position[0]:.1f},{position[1]:.1f})")

        # update fuel consumtipn state if engine is running
        if power > 0 and fuel > 0:
            fuel_used = burn_rate * (power/100) * dt
            fuel_used = min(fuel_used, fuel)  # don't use more fuel than we have
            fuel -= fuel_used
            mass -= fuel_used  # mass decreases as fuel is consumed 
            if fuel <= 0:
                fuel = 0
                print(f"t={time_elapsed:.1f}s: Fuel depleted")
        # Increment time
        time_elapsed += dt
    # display trajectory if requested
    if 'trajectory' in sim['display']:
        display_trajectory()
    # print req reports
    for report in sim['reports']:
        if report == 'max_altitude':
            max_alt = max(point[1] for point in trajectory)
            print(f"Maximum altitude: {max_alt:.1f}m")
        elif report == 'range':
            print(f"Final range: {position[0]:.1f}m")

''' parsing functions '''
# parse rocket properties
# also calculates area based on diameter and returns the rocket mass and thrust
# file must start with rocket { and end with }
def parse_rocket(lines, start_line):
    # goes through the lines until it finds a closing brace and parses the properties
    i = start_line + 1
    while i < len(lines):
        clean = clean_line(lines[i])
        if not clean:
            i += 1
            continue
        if clean == '}':
            break
        # 
        if '=' in clean: # if the line has an equal sign
            prop, value = clean.split('=', 1) # split the line into property and value
            rocket[prop.strip()] = process_value(value) # take vale and add it to the defined rocket dictionary
        i += 1 
    
    rocket['area'] = math.pi * (rocket['diameter']/2)**2 # 
    print(f"Defined rocket: mass={rocket['mass']}kg, thrust={rocket['thrust']}N")
    if rocket.get('fizzbuzz'):
        print("FizzBuzz deployment enabled!")
    return i + 1 # return the next line to parse to use in calling the next parsing function 

# parse environment properties
def parse_environment(lines, start_line):
    # same as parse_rocket, but for environment properties
    # goes through the lines until it finds a closing brace and parses the properties
    # here we can define the gravity if we want to simulate on another planet or something like that 
    i = start_line + 1  
    while i < len(lines):
        clean = clean_line(lines[i])
        if not clean:
            i += 1
            continue
        if clean == '}':
            break
        if '=' in clean:
            prop, value = clean.split('=', 1)
            env[prop.strip()] = process_value(value)
        i += 1
    
    print(f"Defined environment with gravity={env['gravity']}")
    return i + 1 # return the next line to parse to use in calling the next parsing function

# parse flight properties
# sample flight: 
# flight { name = "flight1" at t=0 { angle=90 power=100} at altitude=1000 } or for a time event
# flight { name = "flight1" at t=0 { angle=90 power=100} at t=10 { angle=45 power=50} }
def parse_flight(lines, start_line):
    global flight
    flight_name = clean_line(lines[start_line]).split()[1]  # Extract flight name from already cleaned line
    flight = {'events': []}
    
    i = start_line + 1
    while i < len(lines):
        clean = clean_line(lines[i])
        if not clean:
            i += 1
            continue
        if clean == '}':
            break
        if clean.startswith('at'):
            condition = clean.split('{')[0].strip() # extract the condition from the line 
            condition_parts = condition.split('=') 
            event = {
                'condition_type': condition_parts[0].replace('at', '').strip(), # the condition type (t or altitude)
                'condition_value': float(condition_parts[1].strip()), # the condition value associated to t or alt
                'actions': [] # actions to be taken when the condition is met (set the angle or power)
            }
            action_line = i + 1 
            while action_line < len(lines): # here we parse actions within the event block
                clean_action = clean_line(lines[action_line])
                if not clean_action:
                    action_line += 1
                    continue
                if clean_action == '}':
                    break
                if '=' in clean_action:
                    prop, value = clean_action.split('=', 1)
                    event['actions'].append({
                        'type': 'set', 
                        'property': prop.strip(), # angle or power
                        'value': process_value(value) # processed value (handles percentages for power using the helper function)
                    })
                action_line += 1
            flight['events'].append(event) 
            i = action_line + 1
        else:
            i += 1
    print(f"Defined flight sequence: {flight_name}") 
    return i + 1

# parse simulation properties
def passe_simulation(lines, start_line): 
    sim = {'display': ['trajectory'], 'reports': []} # default display and reports, no reportes and displays trajectory by default
    i = start_line + 1 # clean the line and start parsing as before
    while i < len(lines):
        clean = clean_line(lines[i])
        if not clean:
            i += 1
            continue
        if clean == '}':
            break

    # simple parser to extract desired properties
        if '=' in clean:
            prop, value = clean.split('=', 1)
            sim[prop.strip()] = process_value(value)
        elif clean.startswith('display'):
            displays = clean.replace('display', '').strip().split(',')
            sim['display'] = [d.strip() for d in displays]
        elif clean.startswith('report'):
            reports = clean.replace('report', '').strip().split(',')
            sim['reports'] = [r.strip() for r in reports] 
        i += 1
    
    run_simulation(sim) # this is the main function that runs the simulation, its called later inparse orbit file
    return i + 1

# create in teractive display of the trajectory (text based)
def display_trajectory():
    
    max_height = max(point[1] for point in trajectory)
    max_range = max(point[0] for point in trajectory)
    
    height, width = 20, 80 # grid size
    height_scale = max_height / height if max_height > 0 else 1 # scale for height
    range_scale = max_range / width if max_range > 0 else 1    # scale for range
    grid = [[' ' for _ in range(width+1)] for _ in range(height+1)]
    # draw ground
    for x in range(width+1): grid[height][x] = '-'    
    # plot trajectory
    for x, y in trajectory:
        grid_x = int(x / range_scale) if range_scale > 0 else width # needed to scale the x position to the grid using range scale
        grid_y = height - int(y / height_scale) if height_scale > 0 else height  # needed to scale the y position to the grid using height scale
        if 0 <= grid_x <= width and 0 <= grid_y <= height: # check bounds
            grid[grid_y][grid_x] = '*'
    
    print(f"\nTrajectory (max height: {max_height:.1f}m, range: {max_range:.1f}m)") 
    print("-" * (width + 1)) # make line for the top of the grid
    for y in range(height+1):
        if y == 0:
            print(f"{max_height:.1f}m |{''.join(grid[y])}")
        elif y == height:
            print(f"0m +{''.join(grid[y])}") # add 0m starting line on y
        else:
            print(f"    |{''.join(grid[y])}") # create the y-axis
    print(" " * 7 + "0m" + " " * (width - 5) + f"{max_range:.1f}m") # add 0m starting line on x

# MAIN CALLING FUNCTION
def parse_orbit_file(filename):
    with open(filename, 'r') as file: 
        lines = [line.strip() for line in file.readlines()]
    
    i = 0
    while i < len(lines):
        clean = clean_line(lines[i]) # clean the line and remove comments
        if not clean:
            i += 1
            continue
        if clean.startswith('rocket'): # if the line starts with rocket, we need to parse the rocket properties
            i = parse_rocket(lines, i)
        elif clean.startswith('environment'): # if the line starts with environment, we need to parse the environment properties
            i = parse_environment(lines, i)
        elif clean.startswith('flight'): # if the line starts with flight, we need to parse the flight properties
            i = parse_flight(lines, i)
        elif clean.startswith('simulate'): # if the line starts with simulate, we need to parse the simulation properties
            i = passe_simulation(lines, i)
        else:
            print(f"Unknown statement: {clean}") # handle unknown statements
            i += 1

''' helper functions '''
# clean up file lines - remove comments and empty lines
def clean_line(line):
    line = line.strip()
    if line.startswith('//') or not line: return None
    if '//' in line: line = line.split('//')[0].strip()
    return line

# convert percentage strings to floats for analysis
def process_value(value):
    value = value.strip()
    if value == 'True' or value == 'true':
        return True
    if value == 'False' or value == 'false':
        return False
    if '%' in value: return float(value.replace('%', ''))
    try: return float(value)
    except: return value

# calculate air density based on altitude
# 1.22 kg/m^3 at sea level, decreases by 10% per km up to 14km
def calculate_air_density(altitude):
    if altitude < 1000: 
        return 1.22 
    elif altitude < 14000 and altitude >= 1000:
        return 1.22 * (0.9 ** (altitude / 1000))  # 10% drop per 1000m
    else:
        # beyond 14km, density is negligible (not realistic but needed for simulation)
        return 1.22 * (0.9 ** (14000 / 1000))
    
# main entry point, parses the orbit file and runs the simulation
parse_orbit_file(sys.argv[1])