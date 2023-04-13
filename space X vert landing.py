import krpc
# import math


class NoThrustError(Exception):
  pass

def gravitation(body_mass,equatorial_radius, height):
  Fz = (6.67*(10**-11))*(body_mass) / ((equatorial_radius+height)**2)
  return Fz

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

class Rocket:
  GEAR_EXTEND_TIME = 1 + 2#+margin

  def __init__(self):
    self.landing = True
    self.conn = krpc.connect()
    self.vessel = self.conn.space_center.active_vessel
    self.flight = self.vessel.flight()
    self.control = self.vessel.control
    self.vert_frame = self.vessel.orbital_reference_frame
    self.ref_frame = self.conn.space_center.ReferenceFrame.create_hybrid(
      position=self.vessel.orbit.body.reference_frame,
      rotation=self.vessel.surface_reference_frame
    )
    self.vert_speed = self.conn.add_stream(
      getattr, self.vessel.flight(self.ref_frame), 'vertical_speed'
    )
    self.HEIGHT = self.lowest_point+6.2 #margin
    self.create_simulation()
    self.target = {}
    if self.speed > 10:
      self.control.sas_mode = self.control.sas_mode.retrograde
    else:
      self.control.sas_mode = self.control.sas_mode.stability_assist
  
  def update(self):
    target = self.calc_target(height=self.height, speed=self.speed, mass=self.mass)

    F = (-self.vertical_speed-target['velocity'])*self.vessel.mass + self.vessel.mass*9.81
    if self.vertical_speed > 0:
      F = self.vessel.mass*9.81/2
    throttle = F/self.vessel.max_thrust
    self.control.throttle = throttle#/math.sin(math.radians(pitch()))
    if target["time"] < Rocket.GEAR_EXTEND_TIME and target["time"] > 0:
      self.vessel.control.gear = True
    if self.height <= 0.1 and self.vertical_speed > -2:
      self.landing = False
      self.control.throttle = 0
    if self.speed < 10:
      self.control.sas_mode = self.control.sas_mode.stability_assist
    self.perform_retrograde()
  
  def perform_retrograde(self) -> None:
    # self.control.pitch = -1
    # self.control.jaw = -1
    # self.control.roll = -1
    pass

  def print_data(self):
    print(self.target['time'], self.height, self.vertical_speed, self.target['velocity'])

  def calc_target(self, height, speed, mass):
    floor_height = floor(list(self.simulation.keys()), self.height-speed)
    self.target = self.simulation[floor_height]
    return self.target

  def create_simulation(self):
    self.simulation = {}

    START_SPEED = 3

    h = 0
    v = START_SPEED
    t = 0
    m = self.mass
    thrust = self.max_thrust
    if thrust <= 0:
      raise NoThrustError()
    throttle = 0.5
    delta_t = 0.05
    while h < self.height:
      F = (thrust*throttle) - gravitation(self.vessel.orbit.body.mass, self.vessel.orbit.body.equatorial_radius, h)*self.mass
      if F < 0:
        throttle += 0.1*delta_t
        continue
      v += (F/self.mass) * delta_t

      h += v*delta_t
      if v <= h+5:
        throttle += 0.2*delta_t
      
      throttle = min(0.90, throttle)

      # input([v, h, final_height])
      self.simulation.update({h: {"velocity":v, "time":t}})
      t += delta_t
      print(t, v)
    # return simulation

  @property
  def lowest_point(self):
    bounding_box = self.vessel.bounding_box(self.ref_frame)
    return abs(bounding_box[0][1] - bounding_box[1][1])

  @property
  def height(self):
    return self.flight.surface_altitude - self.HEIGHT
  
  @property
  def speed(self):
    return self.vessel.flight(self.vessel.orbit.body.reference_frame).speed
  
  @property
  def three_axis_velocity(self):
    return self.vessel.flight(self.ref_frame).velocity#(self.vessel.reference_frame)
  
  @property
  def vertical_speed(self):
    return self.three_axis_velocity[0]
  
  @property
  def mass(self):
    return self.vessel.mass
  
  @property
  def max_thrust(self):
    return self.vessel.max_thrust



if __name__ == '__main__':
  rocket = Rocket()
  
  while rocket.landing:
    rocket.update()
    rocket.print_data()