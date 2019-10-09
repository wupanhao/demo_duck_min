__all__ = ['logger','get_current_robot_name','all_disabled','contract','memoize_simple','DuckietownConstants']

import logging
import rospkg

FORMAT = "%(name)15s|%(filename)15s:%(lineno)-4s - %(funcName)-15s| %(message)s"
#
#
# if Logger.root.handlers:  # @UndefinedVariable
#     for handler in Logger.root.handlers:  # @UndefinedVariable
#         if isinstance(handler, StreamHandler):
#             formatter = Formatter(FORMAT)
#             handler.setFormatter(formatter)
# else:
logging.basicConfig(format=FORMAT)

logger = logging.getLogger('DT')
logger.setLevel(logging.DEBUG)


def get_current_robot_name():
    import socket
    robot_name = socket.gethostname()

    return robot_name

def all_disabled():
    return True

def contract(**kwargs):  # @UnusedVariable

    def phi(f):
        return f

    return phi

from decorator import decorator

def memoize_simple(obj):
    cache = obj.cache = {}

    def memoizer(f, *args):
        key = (args)
        if key not in cache:
            cache[key] = f(*args)
        assert key in cache

        try:
            cached = cache[key]
            return cached
        except ImportError:  # pragma: no cover  # impossible to test
            del cache[key]
            cache[key] = f(*args)
            return cache[key]

            # print('memoize: %s %d storage' % (obj, len(cache)))

    return decorator(memoizer, obj)

def indent(s, prefix, first=None):
    s = str(s)
    assert isinstance(prefix, str)
    lines = s.split('\n')
    if not lines: return ''

    if first is None:
        first = prefix

    m = max(len(prefix), len(first))

    prefix = ' ' * (m - len(prefix)) + prefix
    first = ' ' * (m - len(first)) + first

    # differnet first prefix
    res = ['%s%s' % (prefix, line.rstrip()) for line in lines]
    res[0] = '%s%s' % (first, lines[0].rstrip())
    return '\n'.join(res)

def format_obs(d, informal=False):
    """ Shows objects values and typed for the given dictionary """
    if not d:
        return str(d)

    maxlen = 0
    for name in d:
        maxlen = max(len(name), maxlen)

    def pad(pre):
        return ' ' * (maxlen - len(pre)) + pre

    res = ''

    S = sorted(d)
    for i, name in enumerate(S):
        value = d[name]
        prefix = pad('%s: ' % name)
        if i > 0:
            res += '\n'

        s = _get_str(value, informal)

        res += indent(s, ' ', first=prefix)

    return res
def raise_wrapped(etype, e, msg, compact=False, exc=None, **kwargs):
    """ Raises an exception of type etype by wrapping
        another exception "e" with its backtrace and adding
        the objects in kwargs as formatted by format_obs.

        if compact = False, write the whole traceback, otherwise just str(e).

        exc = output of sys.exc_info()
    """

    e = raise_wrapped_make(etype, e, msg, compact=compact, **kwargs)

    if exc is not None:
        _, _, trace = exc
        raise etype, e.args, trace
    else:
        raise e


def raise_wrapped_make(etype, e, msg, compact=False, **kwargs):
    """ Constructs the exception to be thrown by raise_wrapped() """
    assert isinstance(e, BaseException), type(e)
    assert isinstance(msg, (str, unicode)), type(msg)
    s = msg
    if kwargs:
        s += '\n' + format_obs(kwargs)

    if sys.version_info[0] >= 3:
        es = str(e)
    else:
        if compact:
            es = str(e)
        else:
            import traceback
            es = traceback.format_exc(e)

    s += '\n' + indent(es.strip(), '| ')

    return etype(s)
def raise_desc(etype, msg, args_first=False, **kwargs):
    """

        Example:
            raise_desc(ValueError, "I don't know", a=a, b=b)
    """
    assert isinstance(msg, str), type(msg)
    s1 = msg
    if kwargs:
        s2 = format_obs(kwargs)
    else:
        s2 = ""

    if args_first:
        s = s2 + "\n" + s1
    else:
        s = s1 + "\n" + s2

    raise etype(s)

def get_duckiefleet_root():
    return rospkg.RosPack().get_path('duckietown') +  '/duckiefleet'

def get_ros_package_path(package_name):
    """ Returns the path to a package. """
    import rospkg  # @UnresolvedImport
    rospack = rospkg.RosPack()  # @UndefinedVariable
    return rospack.get_path(package_name)


class DTException(Exception):
    """ All exceptions derive from this one. """


class DTUserError(DTException):
    """
        Exceptions that will not be printed with full traceback,
        because they contain a simple message for the user, to be printed in red.
    """


class DTConfigException(DTUserError):
    """
        The configuration (either environment variables or YAML files)
        is invalid.
    """


class DTBadData(DTException):
    """
        A log is invalid.
    """


class DTNoMatches(DTUserError):
    """ Could not find any matches for the user selector """

# XXX: does not represent None as null, rather as '...\n'
def yaml_load(s):
    from ruamel import yaml

    if s.startswith('...'):
        return None
    try:
        l = yaml.load(s, Loader=yaml.RoundTripLoader)
    except:
        l = yaml.load(s, Loader=yaml.UnsafeLoader)

    return remove_unicode(l)


def yaml_load_plain(s):
    from ruamel import yaml

    if s.startswith('...'):
        return None
    l = yaml.load(s, Loader=yaml.UnsafeLoader)
    return remove_unicode(l)


def yaml_dump(s):
    from ruamel import yaml
    res = yaml.dump(s, Dumper=yaml.RoundTripDumper, allow_unicode=False)
    return res


def yaml_dump_pretty(ob):
    from ruamel import yaml
    return yaml.dump(ob, Dumper=yaml.RoundTripDumper)


def remove_unicode(x):

    if isinstance(x, unicode):
        return x.encode('utf8')

    if isinstance(x, dict):
        T = type(x)
        return T([(remove_unicode(k), remove_unicode(v)) for k, v in x.items()])

    if isinstance(x, list):
        T = type(x)
        return T([remove_unicode(_) for _ in x])

    return x

import os

def friendly_path(path, use_environment=True):
    """
        Gets a friendly representation of the given path,
        using relative paths or environment variables
        (if use_environment = True).
    """
    # TODO: send extra rules

    options = []

    options.append(os.path.relpath(path, os.getcwd()))

    rules = []
    rules.append(('~', os.path.expanduser('~')))
    rules.append(('.', os.getcwd()))
    rules.append(('.', os.path.realpath(os.getcwd())))

    if use_environment:
        envs = dict(os.environ)
        # remove unwanted
        for e in list(envs.keys()):
            if 'PWD' in e:
                del envs[e]

        for k, v0 in envs.items():
            if v0:
                for v in [v0, os.path.realpath(v0)]:
                    if v and v[-1] == '/':
                        v = v[:-1]
                    if v[0] == '/':
                        rules.append(('${%s}' % k, v))

    # apply longest first
    rules.sort(key=lambda x: (-len(x[1])))
    path = replace_variables(path, rules)

    options.append(path)

    weight_doubledot = 5

    def score(s):
        # penalize '..' a lot
        s = s.replace('..', '*' * weight_doubledot)
        return len(s)

    options.sort(key=score)
    result = options[0]

    # print('Converted %s  => %s' % (original, result))

    return result


def replace_variables(path, rules):
    for k, v in rules:
        if path.startswith(v):
            # print("  applied %s => %s" % (v, k))
            path = path.replace(v, k)
    return path


def yaml_load_file(filename, plain_yaml=False):
    if not os.path.exists(filename):
        msg = 'File does not exist: %s' % friendly_path(filename)
        raise ValueError(msg)
    with open(filename) as f:
        contents = f.read()
    return interpret_yaml_file(filename, contents,
                               f=lambda _filename, data: data,
                               plain_yaml=plain_yaml)


def interpret_yaml_file(filename, contents, f, plain_yaml=False):
    """
        f is a function that takes

            f(filename, data)

        f can raise KeyError, or DTConfigException """
    try:
        from ruamel.yaml.error import YAMLError

        try:
            if plain_yaml:
                data = yaml_load_plain(contents)
            else:
                data = yaml_load(contents)
        except YAMLError as e:
            msg = 'Invalid YAML content:'
            raise_wrapped(DTConfigException, e, msg, compact=True)
        except TypeError as e:
            msg = 'Invalid YAML content; this usually happens '
            msg += 'when you change the definition of a class.'
            raise_wrapped(DTConfigException, e, msg, compact=True)
        try:
            return f(filename, data)
        except KeyError as e:
            msg = 'Missing field "%s".' % e.args[0]
            raise DTConfigException(msg)

    except DTConfigException as e:
        msg = 'Could not interpret the contents of the file using %s()\n' % f.__name__
        msg += '   %s\n' % friendly_path(filename)
        msg += 'Contents:\n' + indent(contents[:300], ' > ')
        raise_wrapped(DTConfigException, e, msg, compact=True)



def d8n_make_sure_dir_exists(filename):
    """
        Makes sure that the path to file exists, by creating directories.

    """
    dirname = os.path.dirname(filename)

    # dir == '' for current dir
    if dirname != '' and not os.path.exists(dirname):
        d8n_mkdirs_thread_safe(dirname)


def d8n_mkdirs_thread_safe(dst):
    """
        Make directories leading to 'dst' if they don't exist yet.

        This version is thread safe.

    """
    if dst == '' or os.path.exists(dst):
        return
    head, _ = os.path.split(dst)
    if os.sep == ':' and not ':' in head:
        head += ':'
    d8n_mkdirs_thread_safe(head)
    try:
        mode = 511  # 0777 in octal
        os.mkdir(dst, mode)
    except OSError as err:
        if err.errno != 17:  # file exists
            raise

def expand_all(filename):
    """
        Expands ~ and ${ENV} in the string.

        Raises DTConfigException if some environment variables
        are not expanded.

    """
    fn = filename
    fn = os.path.expanduser(fn)
    fn = os.path.expandvars(fn)
    if '$' in fn:
        msg = 'Could not expand all variables in path %r.' % fn
        raise DTConfigException(msg)
    return fn

def write_data_to_file(data, filename):
    """
        Writes the data to the given filename.
        If the data did not change, the file is not touched.

    """
    if not isinstance(data, str):
        msg = 'Expected "data" to be a string, not %s.' % type(data).__name__
        raise ValueError(msg)
    if len(filename) > 256:
        msg = 'Invalid argument filename: too long. Did you confuse it with data?'
        raise ValueError(msg)

    filename = expand_all(filename)
    d8n_make_sure_dir_exists(filename)

    if os.path.exists(filename):
        current = open(filename).read()
        if current == data:
            if not 'assets/' in filename:
                logger.debug('already up to date %s' % friendly_path(filename))
            return

    with open(filename, 'w') as f:
        f.write(data)
    logger.debug('Written to: %s' % friendly_path(filename))

class DuckietownConstants(object):
    DUCKIETOWN_ROOT_variable = 'DUCKIETOWN_ROOT'
    DUCKIETOWN_TMP_variable = 'DUCKIETOWN_TMP'
    DUCKIEFLEET_ROOT_variable = 'DUCKIEFLEET_ROOT'
    DUCKIETOWN_DATA_variable = 'DUCKIETOWN_DATA'
    DUCKIETOWN_CONFIG_SEQUENCE_variable = 'DUCKIETOWN_CONFIG_SEQUENCE'

    # The name of a hypothetical robot used in the unit tests.
    ROBOT_NAME_FOR_TESTS = 'robbie'

    # If the environment variable is not set, use these:
    duckiefleet_root_defaults = [
        '~/duckiefleet',
        '~/duckiefleet-fall2017',
    ]

    # path of machines file inside DUCKIEFLEET_ROOT
    machines_path_rel_to_root = 'catkin_ws/src/00-infrastructure/duckietown/machines'

    use_cache_for_algos = False

    enforce_no_tabs = True
    enforce_naming_conventions = True

    # The rules for good readmes do not apply to these packages
    good_readme_exceptions = ['apriltags_ros', 'apriltags', 'duckietown', 'isam']

    # Show more information about dependencies
    debug_show_package_import_info = False

    ADD_SHORTCUT_TAG = '@create-shortcut-for-this'

    show_timeit_benchmarks = False

def check_isinstance(ob, expected, **kwargs):
    if not isinstance(ob, expected):
        kwargs['object'] = ob
        raise_type_mismatch(ob, expected, **kwargs)