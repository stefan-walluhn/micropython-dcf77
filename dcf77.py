import micropython
import time
from machine import Pin, Timer, disable_irq, enable_irq


def irq_handler(func):
    def stop_start(*args, **kwargs):
        state = disable_irq()
        func(*args, **kwargs)
        enable_irq(state)

    return stop_start


class DCF77Error(ValueError): ...


class InvalidTick(DCF77Error): ...


class DCF77BeaconError(DCF77Error): ...


class ParityError(DCF77BeaconError): ...


class IncompleteBeaconError(DCF77BeaconError): ...


class DCF77Handler:
    def on_tick(self, _):
        pass

    def on_sync(self, _):
        pass

    def on_tick_error(self, _):
        pass

    def on_sync_error(self, _):
        pass


class DCF77Decoder:
    def decode_minute(self, beacon):
        self.raise_on_parity_error((beacon >> 21) & 255)
        return ((beacon >> 21) & 15) + ((beacon >> 25) & 7) * 10

    def decode_hour(self, beacon):
        self.raise_on_parity_error((beacon >> 29) & 127)
        return ((beacon >> 29) & 15) + ((beacon >> 33) & 3) * 10

    def decode_date(self, beacon):
        self.raise_on_parity_error((beacon >> 36) & 8388607)
        return (
            self._decode_year(beacon),
            self._decode_month(beacon),
            self._decode_day(beacon),
        )

    def _decode_day(self, beacon):
        return ((beacon >> 36) & 15) + ((beacon >> 40) & 3) * 10

    def _decode_month(self, beacon):
        return ((beacon >> 45) & 15) + ((beacon >> 49) & 1) * 10

    def _decode_year(self, beacon):
        return 2000 + ((beacon >> 50) & 15) + ((beacon >> 54) & 15) * 10

    def parity_error(self, data):
        data ^= data >> 16
        data ^= data >> 8
        data ^= data >> 4
        data ^= data >> 2
        data ^= data >> 1

        if data & 1:
            return True

        return False

    def raise_on_parity_error(self, data):
        if self.parity_error(data):
            raise ParityError(f"invalid parity on data: {bin(data)}")

    def __call__(self, beacon):
        year, month, day = self.decode_date(beacon)
        return (
            year,
            month,
            day,
            0,
            self.decode_hour(beacon),
            self.decode_minute(beacon),
            0,
            0
        )


class DCF77SyncDetector:
    def __init__(self):
        self.last_tick = 0

    def __call__(self, tick):
        if time.ticks_diff(tick, self.last_tick) > 4000:
            self.last_tick = tick

        sync_tick = time.ticks_diff(tick, self.last_tick) > 1500

        self.last_tick = tick

        return sync_tick


class DCF77:
    def __init__(
        self,
        data_pin,
        enable_pin,
        handler,
        timers=(Timer(0), Timer(1)),
        decode=DCF77Decoder(),
        is_sync_tick=DCF77SyncDetector(),
        min_buffer_size=59,
    ):
        self.enable_pin = enable_pin
        self.data_pin = data_pin
        self.handler = handler

        self.timers = timers
        self.decode = decode
        self.is_sync_tick = is_sync_tick
        self.min_buffer_size = min_buffer_size

        self.reset()

    def reset(self):
        self.__buffer__ = 1

    @property
    def beacon(self):
        if self.__buffer__ < (1 << self.min_buffer_size):
            raise IncompleteBeaconError(
                f"incomplete beacon: {bin(self.__buffer__)}"
            )

        # XXX reduce
        beacon = 0
        for i in range(59):
            beacon = (beacon << 1) + ((self.__buffer__ >> i) & 1)
        return beacon

    def start(self):
        self.reset()
        self.data_pin.irq(trigger=Pin.IRQ_RISING, handler=self.__trigger__)
        self.enable_pin.off()

    def stop(self):
        self.enable_pin.on()
        self.data_pin.irq()

    @irq_handler
    def __trigger__(self, pin):
        self.timers[0].init(
            period=50,
            mode=Timer.ONE_SHOT,
            callback=lambda t: self.__sync__(time.ticks_ms()),
        )

    @irq_handler
    def __sync__(self, tick):
        if not self.data_pin():
            micropython.schedule(self.handler.on_tick_error, InvalidTick())
            return

        try:
            if self.is_sync_tick(tick):
                micropython.schedule(self.handler.on_sync, self.decode(self.beacon))
                self.reset()
        except DCF77BeaconError as error:
            micropython.schedule(self.handler.on_sync_error, error)
            self.reset()

        self.timers[1].init(
            period=(150 - time.ticks_diff(time.ticks_ms(), tick)),
            mode=Timer.ONE_SHOT,
            callback=lambda t: self.__read__()
        )

    @irq_handler
    def __read__(self):
        value = self.data_pin()
        self.__buffer__ = (self.__buffer__ << 1) + value
        micropython.schedule(self.handler.on_tick, value)
