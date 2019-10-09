
__all__ = [
    'Configurable',
    'rospy_timeit_clock',
    'rospy_timeit_wall',
    'timeit_clock',
    'timeit_wall',    
    'bgr_color_from_string',
    'ColorConstants',
    'matplotlib_01_from_rgb',
    'CreateImageFromPylab',
]    
import copy

class Configurable(object):
    """ Utility class to read configuration """

    def __init__(self, param_names, configuration0):

        configuration0 = copy.deepcopy(configuration0)

        if not isinstance(configuration0, dict):
            msg = 'Expecting a dict, obtained %r' % configuration0
            raise ValueError(msg)
        configuration = {}
        configuration.update(configuration0)
        # check that we set all parameters
        given = list(configuration)

        required = list(param_names)

        extra = set(given) - set(required)
        missing = set(required) - set(given)
        if extra or missing:
            msg = ('Error while loading configuration for %r from %r.' %
                   (self, configuration))
            msg += '\n'
            msg += 'Extra parameters: %r\n' % extra
            msg += 'Missing parameters: %r\n' % missing
            raise ValueError(msg)

        assert set(given) == set(required)
        for p in param_names:
            value = configuration[p]
            # if the list is 3 numbers, we convert to array
            if isinstance(value, list) and len(value) == 3:
                import numpy as np
                value = np.array(value)
            configuration[p] = value

        for p in param_names:
            setattr(self, p, configuration[p])

        return configuration



from contextlib import contextmanager
import time

show_timeit_benchmarks = False

@contextmanager
def rospy_timeit_clock(s):
    import rospy
    t0 = time.clock()
    yield
    delta = time.clock() - t0
    rospy.loginfo('%10d ms: %s' % (1000 * delta, s))


@contextmanager
def rospy_timeit_wall(s):
    import rospy
    t0 = time.time()
    yield
    delta = time.time() - t0
    rospy.loginfo('%10d ms: %s' % (1000 * delta, s))


class Stack(object):
    stack = []


@contextmanager
def timeit_generic(desc, minimum, time_function):
#     logger.debug('timeit %s ...' % desc)
    Stack.stack.append(desc)
    t0 = time_function()
    yield
    t1 = time_function()
    delta = t1 - t0
    Stack.stack.pop()
    if minimum is not None:
        if delta < minimum:
            return
    if show_timeit_benchmarks or (minimum is not None):
        pre = '   ' * len(Stack.stack)
        msg = 'timeit_clock: %s %6.2f ms  for %s' % (pre, delta * 1000, desc)
#        t0 = time_function()
        print(msg)
#        t1 = time_function()
#        delta = t1 - t0


@contextmanager
def timeit_clock(desc, minimum=None):
    with timeit_generic(desc=desc, minimum=minimum, time_function=time.clock):
        yield


@contextmanager
def timeit_wall(desc, minimum=None):
    with timeit_generic(desc=desc, minimum=minimum, time_function=time.time) as f:
        yield f







def rgb_color_from_bgr_color(c):
    B, G, R = 0, 1, 2
    return c[R], c[G], c[B]


class ColorConstants(object):
    STR_WHITE = 'white'
    STR_YELLOW = 'yellow'
    STR_RED = 'red'
    STR_BLACK = 'black'
    STR_GRAY = 'gray'
    STR_GREEN = 'green'
    STR_BLUE = 'blue'

    BLACK = (0, 0, 0)  # XXX
    BGR_RED = (0, 0, 255)
    BGR_GREEN = (0, 255, 0)
    BGR_WHITE = (255, 255, 255)
    BGR_BLACK = (0, 0, 0)
    BGR_GRAY = (128, 128, 128)
    BGR_BLUE = (255, 0, 0)
    BGR_YELLOW = (0, 255, 255)
    BGR_DUCKIETOWN_YELLOW = (0, 204, 255)

    RGB_RED = rgb_color_from_bgr_color(BGR_RED)
    RGB_GREEN = rgb_color_from_bgr_color(BGR_GREEN)
    RGB_WHITE = rgb_color_from_bgr_color(BGR_WHITE)
    RGB_BLACK = rgb_color_from_bgr_color(BGR_BLACK)
    RGB_GRAY = rgb_color_from_bgr_color(BGR_GRAY)
    RGB_BLUE = rgb_color_from_bgr_color(BGR_BLUE)
    RGB_YELLOW = rgb_color_from_bgr_color(BGR_YELLOW)
    RGB_DUCKIETOWN_YELLOW = rgb_color_from_bgr_color(BGR_DUCKIETOWN_YELLOW)


def matplotlib_01_from_rgb(c):
    mcolor = tuple(x / 255.0 for x in c)
    return mcolor


def bgr_color_from_string(s):
    d = {
        ColorConstants.STR_YELLOW: ColorConstants.BGR_YELLOW,
        ColorConstants.STR_WHITE: ColorConstants.BGR_WHITE,
        ColorConstants.STR_BLACK: ColorConstants.BGR_BLACK,
        ColorConstants.STR_BLUE: ColorConstants.BGR_BLUE,
        ColorConstants.STR_RED: ColorConstants.BGR_RED,
        ColorConstants.STR_GRAY: ColorConstants.BGR_GRAY,
        ColorConstants.STR_GREEN: ColorConstants.BGR_GREEN,
    }
    if not s in d:
        msg = 'No color %r found in %s' % (s, list(d))
        raise ValueError(msg)
    return d[s]


def segment_color_constant_from_string():
    pass


import tempfile

def bgr_from_png(data):
    """ Returns an OpenCV BGR image from a string """
    s = np.fromstring(data, np.uint8)
    bgr = cv2.imdecode(s, cv2.IMREAD_COLOR)
    if bgr is None:
        msg = 'Could not decode image (cv2.imdecode returned None). '
        msg += 'This is usual a sign of data corruption.'
        raise ValueError(msg)
    return bgr

class CreateImageFromPylab(object):

    def __init__(self, dpi=75, figure_args={}):
        self.dpi = dpi
        suffix = '.png'
        self.temp_file = tempfile.NamedTemporaryFile(suffix=suffix)

        # avoid loading if not necessary
        from matplotlib import pylab

        self.pylab = pylab

        self.figure = self.pylab.figure(**figure_args)

    def __enter__(self):
        return self.pylab

    def __exit__(self, exc_type, exc_value, traceback):  # @UnusedVariable
        if exc_type is not None:
            # an error occurred. Close the figure and return false.
            self.pylab.close()
            return False

        if not self.figure.axes:
#            raise Exception('You did not draw anything in the image.')
            pass

        savefig_params = dict(dpi=self.dpi, bbox_inches='tight', pad_inches=0.01,
                              transparent=True, facecolor=self.figure.get_facecolor())
        self.pylab.savefig(self.temp_file.name, **savefig_params)

        with open(self.temp_file.name) as f:
            self.png_data = f.read()

        self.temp_file.close()

        self.bgr = bgr_from_png(self.png_data)

        self.pylab.close()

    def get_png(self):
        return self.png_data

    def get_bgr(self):
        return self.bgr
