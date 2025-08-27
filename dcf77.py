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


class UncompletedBufferError(DCF77BeaconError): ...


class DCF77Handler:
    def on_tick(self, _):
        pass

    def on_sync(self, _):
        pass

    def on_tick_error(self, _):
        pass

    def on_beacon_error(self, _):
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
            self.decode_hour(beacon),
            self.decode_minute(beacon),
        )


class DCF77CadenceCounter:
    JITTER = 50

    def __init__(self):
        self.reset()

    def reset(self):
        self._cadence = []
        self._calibrating = True

    def _calibrate(self, tick):
        for i, t in enumerate(self._cadence):
            if (
                abs((i + 1) * 1000 - time.ticks_diff(tick, t))
                > DCF77CadenceCounter.JITTER
            ):
                self._cadence = []
                return

        self._cadence.insert(0, tick)
        if len(self._cadence) > 3:
            self._calibrating = False

    def __call__(self, tick):
        if self._calibrating:
            self._calibrate(tick)
            raise InvalidTick

        for i in (1, 2, 3):
            delta_ms = i * 1000 - time.ticks_diff(tick, self._cadence[0])
            if abs(delta_ms) < DCF77CadenceCounter.JITTER:
                self._cadence[0] = tick
                return i

            if delta_ms > 0:
                raise InvalidTick

        self.reset()
        raise InvalidTick


class DCF77:
    def __init__(
        self,
        data_pin,
        enable_pin,
        timer,
        handler,
        decode=DCF77Decoder(),
        counter=DCF77CadenceCounter(),
        min_buffer_size=59,
    ):
        self.enable_pin = enable_pin
        self.data_pin = data_pin
        self.timer = timer
        self.handler = handler

        self.decode = decode
        self.counter = counter
        self.min_buffer_size = min_buffer_size

        self.reset()

    def reset(self):
        self.__buffer__ = 1

    @property
    def beacon(self):
        if self.__buffer__ < (1 << self.min_buffer_size):
            raise UncompletedBufferError(
                f"uncompleted buffer: {bin(self.__buffer__)}"
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
        micropython.schedule(self.__tick__, time.ticks_ms())

    def __tick__(self, tick):
        try:
            if self.counter(tick) > 1:
                self.handler.on_sync(self.decode(self.beacon))
                self.reset()
        except InvalidTick as error:
            self.handler.on_tick_error(error)
            return
        except DCF77BeaconError as error:
            self.handler.on_beacon_error(error)
            self.reset()

        self.timer.init(
            period=(150 - time.ticks_diff(time.ticks_ms(), tick)),
            mode=Timer.ONE_SHOT,
            callback=lambda t: self.__read__(),
        )

    def __read__(self):
        value = self.data_pin()
        self.__buffer__ = (self.__buffer__ << 1) + value
        self.handler.on_tick(value)
