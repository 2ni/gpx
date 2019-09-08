from ppretty import ppretty

class Bunch:
    """
    dummy class
    from http://code.activestate.com/recipes/52308-the-simple-but-handy-collector-of-a-bunch-of-named/?in=user-97991
    """
    def __init__(self, **kwds):
        self.__dict__.update(kwds)

    def __str__(self):
        """ print function """
        return ppretty(self, seq_length=10)

    def __repr__(self):
        """ print function for lists """
        return ppretty(self, seq_length=10)
