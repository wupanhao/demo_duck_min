

def all_disabled():
    return True

def contract(**kwargs):  # @UnusedVariable

    def phi(f):
        return f

    return phi
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
    
def expand_environment(filename):
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

import logging

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