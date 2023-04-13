import math
import json
import krpc

conn = krpc.connect()
vessel = conn.space_center.active_vessel
flight = vessel.flight()
control = vessel.control

ref_frame = conn.space_center.ReferenceFrame.create_hybrid(
    position=vessel.orbit.body.reference_frame,
    rotation=vessel.surface_reference_frame)
vert_frame = vessel.orbital_reference_frame
vert_speed = conn.add_stream(
    getattr, vessel.flight(ref_frame), 'vertical_speed')
pitch = conn.add_stream(
    getattr, vessel.flight(ref_frame), 'pitch')


class NoThrust(Exception):
  pass

def floor(array:list, number:float):
    i = 0
    while True:
        try:
            if number-array[i] > 0:
                i += 1
            else:
              if i == 0:
                return array[0]
              else:
                return array[i-1]
        except IndexError:
            while True:
                try:
                    return array[i]
                except IndexError:
                    i -= 1

def gravitation(body_mass, height):
  Fz = (6.67*10**(-11))*(body_mass) / \
                ((height)**2)
  # print(Fz)
  return Fz

def simulate(thrust, mass, final_height, max_throttle, time_fraction=1):
  simulation = {}

  if thrust == 0:
    raise NoThrust()

  h = 0
  v = 0
  t = 0
  throttle = 0
  min_throttle = 0
  while h < final_height:
    throttle = min(max_throttle, max(min_throttle, h/150))

    F = (thrust*throttle)/mass - gravitation(vessel.orbit.body.mass, vessel.orbit.body.equatorial_radius+h)*vessel.mass
    
    v += F * time_fraction
    v = max(0, v)

    if v > 0:
      h += v * time_fraction
    else:
      min_throttle += 0.05
    
    # input([v, h, final_height])
    simulation.update({h : {"velocity":v, "time":t}})
    t += time_fraction
    print(t, h, v, throttle)
  return simulation

def get_speed():
  return vessel.flight(vessel.orbit.body.reference_frame).speed

def get_target():
  floor_height = floor(list(simulation.keys()), flight.surface_altitude-calculate_lowest_point() - get_speed())
  target = simulation[floor_height]
  # floor_time = floor([t for t in simulation], target["time"]-1)
  # for s in simulation:
  #   if s["time"] == floor_time:
  #     return s
  # target = simulation[floor-time]
  # burn_time = (floor_height-flight.surface_altitude-calculate_lowest_point()) / vert_speed() #DX / V
  # target.update({"burn_time" : burn_time})
  return target

def calculate_lowest_point():
  bounding_box = vessel.bounding_box(ref_frame)
  return abs(bounding_box[0][1] - bounding_box[1][1])

# lowest_point = calculate_lowest_point()

if __name__ == '__main__':
  print('simulating')
  print(vessel.max_thrust)
  print(vessel.mass)
  simulation = simulate(vessel.max_thrust, vessel.mass, flight.surface_altitude, max_throttle=0.9, time_fraction=1)
  print('simulation ended')
  while True:
    
    target = get_target()
    
    F = (get_speed()-target['velocity'])*vessel.mass + gravitation(vessel.orbit.body.mass, vessel.orbit.body.equatorial_radius+flight.surface_altitude)*vessel.mass

    # print(flight.surface_altitude, floor(list(simulation.keys()), flight.surface_altitude), target['velocity'], get_speed(), F, vessel.max_thrust)
    print(-round(target['time']), round(target['velocity']))
    # f.write(json.dumps([flight.surface_altitude, floor(list(simulation.keys()), flight.surface_altitude), target['velocity'], get_speed(), F, vessel.max_thrust]) + '\n')
    throttle = F/vessel.max_thrust if vert_speed() < -0.9 else 0
    control.throttle = throttle#/math.sin(math.radians(pitch()))

    if target["time"] < 5.1 and target["time"] > 0:
      vessel.control.gear = True