"""
Servo Controller to run in a microcontroller


Design Notes.

There are two tasks that each run on their own core and execute independently.
Micropython in RP2040 does not have a global lock and allows true multitasking
with a task per core.

Task 0.
This is the initial task after Micropython boots.  It does the following:
1.  initialization and setup
2.  create the second task
3.  Runs a command processing loop that can block while waiting for input
    on the serial interface the connects to the Linux computer.  The loop
    reads one "line" from the interface, looks at the command inside then
    cas the proper function to process that command.   Commands include thins
    like setting the angle of the servos or setting calibration numbers.

Task 1.
This task does several operations that can not block.
1.  If enabled, move the servos at a specified rate.  This allows the servos
    to move smoothly without sonstant input from te Linux computer
2.  Blink the onboard LED
3.  In future revisions...?

"""

import micropython
import _thread
import gc
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

data_lock = _thread.allocate_lock()

led = Pin(25, Pin.OUT)


def get_next_command():
    """returns a full command or None"""

    # TODO Ths needs to come over a different interface such as UART, or SPI
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
    global data_lock
    global cpu_total_us
    global cpu_num_cycles

    op = command[0]

    if op == 'none':
        pass

    elif op == 'angle':
        with data_lock:
            sd.set_angles(command[1])

    elif op == 'rate':
        with data_lock:
            sd.set_rate(command[1])

    elif op == 'cal':
        with data_lock:
            sd.set_calibration(command[1], command[2])

    elif op == 'limit':
        with data_lock:
            sd.set_us_limits(command[1], command[2])

    elif op == 'channel_active':
        with data_lock:
            sd.set_channel_active(command[1])

    elif op == 'start':
        with data_lock:
            sd.start()

    elif op == 'stop':
        with data_lock:
            sd.stop()

    elif op == 'zero':
        with data_lock:
            sd.zero_all()

    elif op == 'echo':
        print('>>>' + str(command) + '<<<')

    elif op == "ID":
        print(version_id)

    elif op == "dump":
        with data_lock:
            print(sd.dump_all())

    elif op == "cpu":
        print("loop uSec", cpu_total_us / cpu_num_cycles)
        print("mem free ", gc.mem_free())

    else:
        pass


@micropython.native
def past_ms(tickval_ms) -> bool:

    delta = utime.ticks_diff(utime.ticks_ms(), tickval_ms)
    if delta > 0:
        return True
    else:
        return False


# CPU utilization tracking, task1
cpu_total_us = 0
cpu_num_cycles = 0


def task1():
    """Advances the servos at given rate and otheR non-blocking actions"""

    global sd
    global data_lock
    global led
    global cpu_total_us
    global cpu_num_cycles



    heartbeat_period        = micropython.const(2000)   # ms
    heartbeat_on_ms         = micropython.const( 400)   # ms
    servo_advance_period    = micropython.const(  50)   # ms

    # init to 1ms in the past.
    servo_advance_next  = utime.ticks_add(utime.ticks_ms(), -1)
    heartbeat_next      = utime.ticks_add(utime.ticks_ms(), -1)
    heartbeat_off       = utime.ticks_add(utime.ticks_ms(),  1)

    while True:
        start_loop = utime.ticks_us()

        if past_ms(heartbeat_next):
            led.on()
            heartbeat_next = utime.ticks_add(heartbeat_next,
                                             heartbeat_period)
            heartbeat_off  = utime.ticks_add(utime.ticks_ms(),
                                             heartbeat_on_ms)
        if past_ms(heartbeat_off):
            led.off()

        if past_ms(servo_advance_next):

            # If this attempt to get the lock fail, it will
            # be retried on the next time around the loop.
            with data_lock:
                sd.advance()
            servo_advance_next = utime.ticks_add(utime.ticks_ms(),
                                                 servo_advance_period)

        # gc.collect() this takes 5 microseconds
        # TODO place call to WDT.feed() here
        cpu_total_us   += utime.ticks_diff(utime.ticks_us(), start_loop)
        cpu_num_cycles += 1
        utime.sleep_ms(5)


def task0():
    """Read and process commands from serial data link"""

    command_poll_period = micropython.const(5)  # ms

    while True:

        command = get_next_command()
        if command is not None:
            exec_command(command)

        # gc.collect() takes too long to run, perhaps move it
        utime.sleep_ms(command_poll_period)

def main():
    """Entry point for servo controller"""

    global led

    # Fash LED fast for "wakeup"
    for i in range(5):
        led.on()
        utime.sleep_ms(100)
        led.off()
        utime.sleep_ms(400)

    # Start the Second thread.
    _thread.start_new_thread(task1, ())

    # Run task0 i the current thread
    task0()


main()
