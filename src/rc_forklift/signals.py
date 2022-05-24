class SignalManager:
    def __init__(self):
        self._signals = {}

    def create_signal(self, key, value = None):
        self._signals[key] = value

    def create_signals(self, keys):
        for key in keys:
            self.create_signal(key)

    def set_signal(self, key, value):
        if key in self._signals: self.create_signal(key, value)

    def get_signal(self, key):
        if key in self._signals: return self._signals[key]
        else: return None
