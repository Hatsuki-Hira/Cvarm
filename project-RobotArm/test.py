import wiringpi, time


pin = 10
wiringpi.wiringPiSetup()
wiringpi.pinMode(pin, wiringpi.GPIO.PWM_OUTPUT)

range_min = 0.025  # %
range_max = 0.125    # %


while 1:
    arr = 1000
    wiringpi.pwmSetRange(pin, arr)
    div = 50
    wiringpi.pwmSetClock(pin, div)
    ccr = int(arr * range_min)
    wiringpi.pwmWrite(pin, ccr)


