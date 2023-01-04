# Class to handle PWM servos

from machine import Pin, PWM
import utime

class ServoDriver:
    def __init__(self, max_channels: int = 4):

        self.max_channels = max_channels
        
        # is the channel active or disabled.  Channels my be enabled or
        # disabled in realtime
        self.channel_active = [True for i in range(max_channels)]

        # rad_target is the target joint angle in radians, in the
        # robot's joint coordinate system.  rad_rate is the angular
        # velocity in radians per second that the joint is to continue
        # to move after reaching rad_target.
        self.rad_target = [0.0 for i in range(max_channels)]
        self.rad_rate   = [0.0 for i in range(max_channels)]

        # Capture the time (ms past epoc) when self.red_target was set
        # we use this along with self.rad_rate to extrapolate rad_target
        # into the future.
        self.rad_target_timestamp_ms = None

        # this linear function maps radians in the robot's joint coordinates
        # to micro seconds to be sent to the servo.  Note that the direction
        # of rotations will change if the slope is negative
        self.cal_slope      = [1.0 for i in range(max_channels)]
        self.cal_intercept  = [0.0 for i in range(max_channels)]
        
        # range of movement limits in microseconds
        self.us_min = [ 500.0 for i in range(max_channels)]
        self.us_max = [2500.0 for i in range(max_channels)]
        
        # store the last uSec values written to the PWM device.  The device is
        # only updated if the value has changed by some threshold from the
        # last time the value was written.
        self.us_last    = [0.0 for i in range(max_channels)]

        # this is the threshold value in uSec, used with self.us_last to
        # determine if a new value should be written to a PWM device.
        self.update_threshold = 0.5
        
        # self.update_period_ms = round(1000 / 20)# period in milliseconds
        
        # If True this driver will actively update the PWM devices at a periodic rate
        # even when no command comes from the user.  This allows for the "rad_rate" to
        # implemented so that servos can move at a specified rate.
        self.active = False
        
        # We use pin0 for channel 0, pin1 for channel 1, and so on to pin15
        # In other words to pin number matches the servo index
        self.pwms = []

        # Assign physical pins to servo channels.  In this example the
        # assignment  in one to one.  But the anything is allowed.
        # It depends on how the wires are plugged in.
        pin_assignments = (0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15)
        for pin_number in pin_assignments:
            self.pwms.append(PWM(Pin(pin_number)))

        self.pwm_freq = 300.0
        for p in self.pwms:
            p.freq(round(self.pwm_freq))

        self.zero_all()

        ## TODO Start the real time loop as a task on second CPU core if active is true.

    def update_loop(self):
        """updates all PWM values, loop runs forever, obviously in a task"""

        ## TODO This will have to run as a task on the second CPU core
        while True:

            ## TODO get lock
            if self.active == True:
                self.advance()
                ## TODO release lock
            utime.sleep_ms (self.update_period_ms)

    def advance(self):
        """set the PWM values based of rad_rate and elapsed time"""
        if self.active and (self.rad_target_timestamp_ms is not None):

            elapsed = utime.ticks_diff(utime.ticks_ms(),
                                       self.rad_target_timestamp_ms) / 1000.0
            assert(elapsed >= 0)

            angles = [self.rad_target[chan] + (elapsed * self.rad_rate[chan])
                      for chan, _ in enumerate(self.rad_target)]
            self.update_once(angles)

    def update_once(self, radians):
        """updates all PWM values"""

        for chan, rad in enumerate(radians):

            # The liner function transforms radians (in robot joint frame)
            # to micro seconds of pulse width and applies measured calibrations
            # for each servo.
            slope: float     = self.cal_slope[chan]
            intercept: float = self.cal_intercept[chan]
            usec: float      = (rad * slope) + intercept

            usec = min(self.us_max[chan], usec)
            usec = max(self.us_min[chan], usec)

            # Has the target angle moved enough from the last time it was
            # set?  There is no point in moving a very tiny distance
            if abs(usec - self.us_last[chan]) > self.update_threshold:

                # The Micropython PWM libray wants an integer number of nano seconds
                self.pwms[chan].duty_ns(round(usec * 1000.0))
                self.us_last[chan] = usec


    def start(self):
        ## TODO get lock
        self.ns_last = [0.0 for i in range(self.max_channels)]
        self.active = True
        ## TODO release lock

    def stop(self):
        ## TODO get lock
        self.active = False
        ## TODO release lock

    def zero_all(selfself):
        """Set the PWM vale for all servos to zero"""
        for chan in range(self.max_channels)
            self.pwms[chan].duty_ns(0)

    def set_angles(self, angles):

        if self.active:
            ## TODO get lock
            for chan in range(self.max_channels):
                self.rad_target = angles
            ## TODO release lock
        else:
            self.rad_target = angles
            self.update_once(angles)

        self.rad_target_timestamp_ms = utime.ticks_ms()

    def set_calibration(self, slope, intercept):

        self.cal_slope      = slope
        self.cal_intercept  = intercept

    def set_us_limits(self, min_lim, max_lim):

        self.us_min = min_lim
        self.us_max = max_lim

    def set_rate(self, angular_rate):

        self.rad_rate = angular_rate

    def set_channel_active(self, act):

        self.channel_active = act
