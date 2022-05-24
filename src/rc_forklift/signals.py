class Signal:
    def __init__(self, name):
        self._name = name
        self._latest_value = False
        self._oldest_value = False

    def update(self):
        self._oldest_value = self._latest_value

    def trigger(self):
        self._latest_value = not self._latest_value

    def is_rising(self) -> bool:
        return self._latest_value != self._oldest_value


class SignalManager:
    def __init__(self):
        self._signals = {}

    def add_signal(self, signal):
        self._signals[signal._name] = signal

    def create_signal(self, name):
        self.add_signal(Signal(name))

    def create_signals(self, name_array):
        for name in name_array:
            self.create_signal(name)
        
    def update(self):
        for signal in self._signals.values():
            signal.update()

    def trigger(self, key):
        if key in self._signals:
            self._signals[key].trigger()

    def is_rising(self, key) -> bool:
        if key in self._signals: return self._signals[key].is_rising()
        else: return False
