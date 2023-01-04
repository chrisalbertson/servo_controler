# Servo Controller to run in a microcontroller

import micropython
from machine import Pin
import utime
import json
import servo_driver

## TODO take comments of the numpy import
"""
try:
    from ulab import numpy as np
except ImportError:
    import numpy as np
"""

version_id = micropython.const("Servo Controller, Version 0.1")

# level 0, assertions are compiled in.  Set to 1 to compile out
micropython.opt_level(0)

# save some bytes for processing the "out of memory exception
micropython.alloc_emergency_exception_buf(100)

# global instance of servo driver
sd = servo_driver.ServoDriver()

def get_next_command():
    """returns a full command or None"""

    # TODO Ths needs to come over a different interface
    line = input('$$$')
    try:
        command = json.loads(line)
    except:
        print('ERROR. json.load() failed')
        command = None

    return command


def exec_command(command):
    """get and exec a command.

    Example of all valid commands
        ('none',)
        ('angle',[745, 2000, 1500,...])
        ('cal',[200.4, 544, 643,...], [456, 34, 77])
        ('start',)
        ('stop',)
        ('echo',)
        ('ID',)
    """

    global version_id
    global sd

    op = command[0]

    if op == 'none':
        pass

    elif op == 'angle':
        sd.set_angles(command[1])

    elif op == 'rate':
        sd.set_rate(command[1])

    elif op == 'cal':
        sd.set_calibration(command[1], command[2])

    elif op == 'cal':
        sd.set_us_limits(command[1], command[2])

    elif op == 'channel_active':
        sd.set_channel_active(command[1])

    elif op == 'start':
        sd.start()

    elif op == 'stop':
        sd.stop()

    elif op == 'zero':
        sd.zero_all()

    elif op == 'echo':
        print('>>>' + str(command) + '<<<')

    elif op == "ID":
        print(version_id)

    else:
        pass


@micropython.native
def past_ms(tickval_ms) -> bool:

    delta = utime.ticks_diff(utime.ticks_ms(), tickval_ms)
    if delta > 0:
        return True
    else:
        return False


def main():
    """ Entry point to servo controller. """

    led = Pin(25, Pin.OUT)
    command_poll_period = micropython.const(10)     # ms
    heartbeat_period    = micropython.const(2000)   # ms



    # init to 1ms in the past.
    heartbeat_next = utime.ticks_add(utime.ticks_ms(), -1)
    heartbeat_off  = utime.ticks_add(utime.ticks_ms(),  1)

    # Fash LED fast for "wakeup"
    for i in range(3):
        led.on()
        utime.sleep_ms(400)
        led.off()
        utime.sleep_ms(600)

    # TODO TEST ONLY REMOVE LATER
    advance_next = utime.ticks_ms()

    while True:

        if past_ms(heartbeat_next):
            led.on()
            heartbeat_next = utime.ticks_add(heartbeat_next,
                                             heartbeat_period)
            heartbeat_off  = utime.ticks_add(utime.ticks_ms(),
                                             1000)
        if past_ms(heartbeat_off):
            led.off()

        # TODO TEST ONLY REMOVE LATER
        if past_ms(advance_next):
            sd.advance()
            advance_next = utime.ticks_add(utime.ticks_ms(),
                                           1000)

        command = get_next_command()
        if command is not None:
            exec_command(command)

        # TODO place call to WDT.feed() here
        utime.sleep_ms(command_poll_period)


main()
