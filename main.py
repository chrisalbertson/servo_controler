# Servo Controller to run in a microcontroller

from machine import Pin
import utime
import servo_driver

## TODO take comments of the numpy import
"""
try:
    from ulab import numpy as np
except ImportError:
    import numpy as np
"""



def get_next_command():
    """returns a full command or None"""
    line = input('$$$')
    command = eval(line)
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

    command = get_next_command()
    print('      got', command)
    op = command[0]

    if op == 'none':
        pass

    elif op == 'angle':
        sd.set_angles(command[1])

    elif op == 'cal':
        sd.set_calibration(command[1], command[2])

    elif op == 'start':
        sd.start()

    elif op == 'stop':
        sd.stop()

    elif op == 'echo':
        print('>>>' + str(command) + '<<<')

    elif op == "ID":
        print(version_id)

    else:
        pass


def past_ms(tickval_ms: _Ticks):

    delta = utime.ticks_diff(utime.ticks_ms(), tickval_ms)
    if delta < 0:
        return True
    else:
        return False


def main():
    print('test 4')

    led = Pin(25, Pin.OUT)
    command_poll_period = 10    # ms
    heartbeat_period = 2000     # ms



    # init to 1ms in the past.
    heartbeat_next = utime.ticks_add(utime.ticks_ms(), -1)
    heartbeat_off  = utime.ticks_add(utime.ticks_ms(),  1)

    # Fash LED fast for "wakeup"
    for i in range(3):
        led.on()
        utime.sleep_ms(400)
        led.off()
        utime.sleep_ms(600)

    while True:

        if past_ms(heartbeat_next):
            led.on()
            heartbeat_next = utime.ticks_add(heartbeat_next,
                                             heartbeat_period)
            heartbeat_off  = utime.ticks_add(utime.ticks_ms(),
                                             1000)
        if past_ms(heartbeat_off):
            led.off()

        command = get_next_command()
        if command is not None:
            exec_command(command)

        utime.sleep_ms(command_poll_period)


"""
def example_sender()
    command = ('angle', [2000, 2300, 900, 1450])
    command_str = str(command)
    stream.write(command_str + '\n')
"""

version_id = "Servo Controller, Version 0.0"
sd = servo_driver.ServoDriver()

main()
