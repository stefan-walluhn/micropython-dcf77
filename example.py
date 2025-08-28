import micropython
from machine import Pin, Timer, RTC
from dcf77 import DCF77, DCF77Handler

micropython.alloc_emergency_exception_buf(100)


class PrintHandler(DCF77Handler):
    def on_tick(self, value):
        print(value, end="")

    def on_sync(self, timestamp):
        print(timestamp)

    def on_sync_error(self, error):
        print(str(error))


class RTCHandler(DCF77Handler):
    def __init__(self, rtc):
        self.rtc = rtc

    def on_sync(self, timestamp):
        print("syncing clock")
        self.rtc.datetime(timestamp)

    def on_sync_error(self, error):
        print(str(error))


dcf = DCF77(Pin(34, Pin.IN), Pin(33, Pin.OUT), Timer(0), RTCHandler(RTC()))
dcf.start()
