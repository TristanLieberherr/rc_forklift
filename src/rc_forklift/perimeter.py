import math
import threading
import numpy as np



def perp(a):
    b = np.empty_like(a)
    b[0] = -a[1]
    b[1] = a[0]
    return b

def intersect_point(a1, a2, b1, b2):
    da = a2-a1
    db = b2-b1
    dp = a1-b1
    dap = perp(da)
    denom = np.dot(dap, db)
    num = np.dot(dap, dp)
    return (num/denom)*db + b1

def ccw(A, B, C) -> bool:
    return (C[1]-A[1]) * (B[0]-A[0]) > (B[1]-A[1]) * (C[0]-A[0])

def intersect(p1, p2, p3, p4) -> bool:
    return ccw(p1, p3, p4) != ccw(p2, p3, p4) and ccw(p1, p2, p3) != ccw(p1, p2, p4)


class Segment:
    def __init__(self, p1, p2):
        self._p1 = np.array(p1)
        self._p2 = np.array(p2)
        self._thetas = None
        self._ranges = None

    def init(self, thetas):
        range_max = 1000    # Check if OK
        p1 = np.array([0, 0])
        p2 = np.array([0, 0])
        self._thetas = thetas
        self._ranges = np.zeros(len(thetas))
        for i, angle in enumerate(self._thetas):
            p2[0] = range_max*math.cos(angle)
            p2[1] = range_max*math.sin(angle)
            if intersect(p1, p2, self._p1, self._p2):
                p = intersect_point(p1, p2, self._p1, self._p2)
                self._ranges[i] = np.linalg.norm(p)
            else:
                self._ranges[i] = np.inf

    def get_ranges(self):
        return self._ranges

class Perimeter:
    def __init__(self):
        self._DISPLAY_OFFSET = np.pi/2
        self._TRESPASS_THRESH = 0
        self._trespassing = 0
        self._range_min = 0
        self._ready = False
        self._shapes = []
        self._thetas = None
        self._ranges = None
        self._scan_ranges = None
        self._lock = threading.Lock()

    def init(self, scan):
        length = len(scan.ranges)
        self._range_min = scan.range_min
        self._thetas = np.zeros(length)
        self._ranges = np.full(length, np.inf)
        self._scan_ranges = np.array(scan.ranges)
        for i, range in enumerate(scan.ranges):
            self._thetas[i] = i*scan.angle_increment + scan.angle_min + self._DISPLAY_OFFSET
        for shape in self._shapes:
            shape.init(self._thetas)
            self._ranges = np.minimum(self._ranges, shape.get_ranges())
        self._ranges[self._ranges == np.inf] = 0
        self._ready = True
        
    def add_shape(self, shape):
        self._shapes.append(shape)

    def create_segment(self, p1, p2):
        self.add_shape(Segment(p1, p2))

    def get_ranges(self):
        return self._ranges

    def get_scan_ranges(self):
        return self._scan_ranges

    def get_thetas(self):
        return self._thetas

    def get_lock(self) -> threading.Lock:
        return self._lock

    def update(self, scan):
        self._lock.acquire()
        if not self._ready:
            self.init(scan)
        self._scan_ranges = np.array(scan.ranges)
        self._scan_ranges[np.isnan(self._scan_ranges)] = 0
        self._scan_ranges[self._scan_ranges < self._range_min] = np.inf
        self._trespassing = np.count_nonzero(self._scan_ranges < self._ranges)
        self._lock.release()

    def is_trespassing(self) -> bool:
        return self._trespassing > self._TRESPASS_THRESH

    def is_ready(self) -> bool:
        return self._ready
