import micropython
from machine import Pin, Timer


class DCF77Error(ValueError): ...


class InvalidTick(DCF77Error): ...


class DCF77BeaconError(DCF77Error): ...


class ParityError(DCF77BeaconError): ...


class IncompleteBeaconError(DCF77BeaconError): ...


class DCF77Handler:
    def on_tick(self, value): ...

    def on_sync(self, timestamp): ...

    def on_sync_error(self, error) -> None:
        raise error


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
            0,
        )


class DCF77:
    def __init__(
        self,
        data_pin,
        enable_pin,
        handler,
        timer=Timer(0),
        decode=DCF77Decoder(),
        min_buffer_size=59,
    ) -> None:
        self.data_pin = data_pin
        self.enable_pin = enable_pin
        self.handler = handler
        self.timer = timer
        self.decode = decode
        self.min_buffer_size = min_buffer_size

        self.__poll_count__ = 0
        self.__current__ = 0
        self.__last__ = 0
        self.__buffer__ = 1 << 60

    @property
    def beacon(self):
        if self.__buffer__ < (1 << self.min_buffer_size):
            raise IncompleteBeaconError(f"incomplete beacon: {bin(self.__buffer__)}")

        # XXX reduce
        beacon = 0
        for i in range(59):
            beacon = (beacon << 1) + ((self.__buffer__ >> i) & 1)
        return beacon

    def reset(self):
        self.__poll_count__ = 0
        self.__buffer__ = 1

    def start(self):
        self.reset()
        self.enable_pin.off()
        self.timer.init(period=50, mode=Timer.PERIODIC, callback=self.__poll__)

    def stop(self):
        self.timer.deinit()
        self.enable_pin.on()

    def __tick__(self, current):
        self.__buffer__ = (self.__buffer__ << 1) + current
        self.handler.on_tick(current)

    def __sync__(self, _):
        try:
            self.handler.on_sync(self.decode(self.beacon))
        except DCF77BeaconError as error:
            self.handler.on_sync_error(error)
        self.reset()

    def __poll__(self, _):
        self.__current__ = self.data_pin()

        if self.__current__ == 0 and self.__last__ == 1:
            if self.__poll_count__ > 2:
                micropython.schedule(self.__tick__, 1)
            else:
                micropython.schedule(self.__tick__, 0)

            self.__poll_count__ = 0

        if self.__current__ == 1 and self.__last__ == 0:
            self.__poll_count__ = 0

        self.__last__ = self.__current__
        self.__poll_count__ += 1

        if self.__poll_count__ > 30:
            micropython.schedule(self.__sync__, None)
            self.__poll_count__ = 0
