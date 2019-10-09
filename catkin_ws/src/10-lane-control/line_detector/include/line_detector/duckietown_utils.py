import copy

__all__ = [
    'Configurable',
    'rospy_timeit_clock',
    'rospy_timeit_wall',
    'timeit_clock',
    'timeit_wall',    
]

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

