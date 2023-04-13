from matplotlib import pyplot as plt
import math
import krpc
import time
# from simple_pid import PID


def nearest(number, array):
    i = 0
    while True:
        try:
            if number-array[i] > 0:
                i += 1
            else:
                return array[i-1]
        except IndexError:
            # print('ERROR')
            while True:
                try:
                    return array[i]
                except IndexError:
                    i -= 1


simulation_time_delta = 0.1
thrust = 0.9


success = False
print('[Simulating]')

def simulate():
    while not success:
        v = 5
        h = 0
        t = 0
        data = {}
        conn = krpc.connect()
        vessel = conn.space_center.active_vessel
        m = vessel.mass
        f = vessel.max_thrust  # vessel.thrust*100
        fuel_consumption = 15  # vessel.flight().thrust_specific_fuel_consumption
        while h < 100000 and not(h == 0 and t > 50000):
            t += simulation_time_delta
            F = (f*thrust)/m
            Fz = (6.67*10**(-11))*(vessel.orbit.body.mass) / \
                ((vessel.orbit.body.equatorial_radius+h)**2)

            if h < 10:
                v = 1
            elif h < 100:
                v = 5
            elif h < 200:
                v = 15
            else:
                v += (F-Fz)*simulation_time_delta
            # if h < 500:
            #    v = (h+1)/4
            #v -= Fz*simulation_time_delta
            if v < 0:
                m -= 100
                v = 0
                
            h += v*simulation_time_delta
            m -= fuel_consumption*thrust*simulation_time_delta
            # v += F
            data.update({h: [t, v]})
            if list(data.keys())[-1] < 100:
                pass
                #print('simulation failed, not enough thrust to take off')
                # vessel.control.activate_next_stage()
            else:
                success = True
    return data

data = simulate()
# x-axis values
x = list(data.keys())

# Y-axis values
y = [data[key][1] for key in list(data.keys())]

# Function to plot scatter
# plt.scatter(x, y)

# function to show the plot
# plt.show()
print('Success')
print('[Preparation]')
obt_frame = vessel.orbit.body.non_rotating_reference_frame
vert_frame = vessel.orbital_reference_frame
ref_frame = conn.space_center.ReferenceFrame.create_hybrid(
    position=vessel.orbit.body.reference_frame,
    rotation=vessel.surface_reference_frame)

orb_speed = conn.add_stream(getattr, vessel.flight(obt_frame), 'speed')
altitude = conn.add_stream(
    getattr, vessel.flight(obt_frame), 'surface_altitude')
vert_speed = conn.add_stream(
    getattr, vessel.flight(ref_frame), 'vertical_speed')
pitch = conn.add_stream(
    getattr, vessel.flight(ref_frame), 'pitch')

print(vert_speed())


#vessel.control.speed_mode = vessel.control.speed_mode.orbit
try:
    ap = vessel.auto_pilot
    ap.sas = True
    ap.rcs = True
    ap.sas_mode = ap.sas_mode.retrograde
except RuntimeError:
    print('[SAS] Failed')

current_thrust = 0
pid_enabled = False

vessel.control.throttle = 0
print('analizing')
atmospheric_height = 0
if vessel.orbit.body.has_atmosphere:
    while vessel.orbit.body.density_at(atmospheric_height) > 0.00035:
        atmospheric_height += 100
print(atmospheric_height)

vessel.control.gear = False
print('success')
while altitude() > 0:
    if vessel.orbit.body.has_atmosphere:
        if vessel.orbit.time_to_apoapsis < 20:
            while vessel.orbit.periapsis_altitude > atmospheric_height:
                print('periapsis')
                vessel.control.throttle = 1000/f
                print(5000/f)

            vessel.control.throttle = 0

        elif vessel.orbit.time_to_periapsis < 20:
            apoapsis = vessel.orbit.apoapsis_altitude
            while apoapsis > vessel.orbit.periapsis_altitude and vessel.orbit.periapsis_altitude > 0:
                print('apoapsis')
                vessel.control.throttle = 1000/f
            vessel.control.throttle = 0
    else:
        if vessel.orbit.time_to_apoapsis < 10:
            while vessel.orbit.periapsis_altitude > 0:
                print('apoapsis')
                vessel.control.throttle = 1000/f

    time, target_speed = data[nearest(
        max(0.1, altitude()-vert_speed()-15), list(data.keys()))]
    target_speed = max(0.1, target_speed-10) #

    # if abs(target_speed-srf_speed()) < 10:
    #    pid_enabled = True
    # if pid_enabled:
    # if altitude() < 100:
    #    target_speed = 5
    for part in vessel.parts.all:
        if part.skin_temperature/part.max_skin_temperature > 0.7:
            target_speed = vert_speed()-(part.temperature/part.max_temperature*10)
        #print(part.skin_temperature, part.max_skin_temperature)
    try:
        slower = (target_speed-abs(vert_speed()))

        thrust = slower/math.sin(math.radians(pitch()))
        thrust = -(thrust * vessel.mass)/vessel.max_thrust
    except ZeroDivisionError:
        pass
    # else:
    #    pid_value = 0
    current_thrust = max(0, min(1, thrust))

    # if altitude() < 10 or vert_speed() > 0:
    #     current_thrust = 0

    print("target speed", round(target_speed), "thrust",
          round(current_thrust), "time", round(-time), "speed", int(vert_speed()), "ap", vessel.orbit.time_to_apoapsis, "per", vessel.orbit.time_to_periapsis)
    vessel.control.throttle = current_thrust
    try:
        if time < 25:
            # ap.sas_mode = ap.sas_mode.stability_assist
            vessel.control.gear = True
        else:
            ap.sas_mode = ap.sas_mode.retrograde
            vessel.control.gear = False
    except:
        print('!')
