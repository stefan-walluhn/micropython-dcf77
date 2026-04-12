import micropython
from machine import RTC, Pin, Timer

from dcf77 import DCF77, DCF77Handler

micropython.alloc_emergency_exception_buf(100)


class PrintHandler(DCF77Handler):
    def on_tick(self, dcf77, value):
        print(value, end="")

    def on_sync(self, dcf77, datetime):
        print(datetime)

    def on_sync_error(self, dcf77, error):
        print(str(error))


class RTCHandler(DCF77Handler):
    def __init__(self, rtc):
        self.rtc = rtc

    def on_sync(self, dcf77, datetime):
        print("syncing clock")
        self.rtc.datetime(datetime)

    def on_sync_error(self, dcf77, error):
        print(str(error))


dcf = DCF77(
    Pin(34, Pin.IN, Pin.PULL_UP), Pin(33, Pin.OUT), PrintHandler(), timer=Timer(1)
)
dcf_rtc = DCF77(Pin(34, Pin.IN), Pin(32, Pin.OUT), RTCHandler(RTC()), timer=Timer(1))
