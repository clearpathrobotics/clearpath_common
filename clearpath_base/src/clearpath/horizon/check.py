
from collections import namedtuple


class Fields(object):
    def __repr__(self):
        '''Get the namedtuple's repr, and replace the name with the current classname.
        This permits payloads to inherit from intermediary abstract payloads.'''
        parent_repr = super(Fields, self).__repr__()
        name, sep, fields = parent_repr.partition('_(')
        return "%s(%s" % (self.__class__.__name__, fields)

    def hex(self):
        return ' '.join(map(lambda c: "%02X" % c, self.data()))

    def check(self, fields):
        names = fields.split()
        return self.Check(self.__class__.__name__, names, 
                          map(lambda name: getattr(self, name), names))

    class Check(object):
        def __init__(self, subjname, names, values):
            self.names = names
            self.values = values
            self.subjname = subjname

        def _check(self, constraint, explanation):
            for name, value in zip(self.names, self.values):
                if constraint(value):
                    e = "%s initialized with %s=%s: %s" % (self.subjname, name, repr(value), explanation)
                    raise ValueError(e)

        def range(self, lowerbound, upperbound):
            self._check(lambda x: x < lowerbound or x > upperbound,
                        "Outside of allowed range [%0.1f,%0.1f]" % 
                        (lowerbound, upperbound))

        def length(self, lowerbound, upperbound):
            self._check(lambda x: len(x) < lowerbound, 
                        "Length shorter than %d" % lowerbound)
            self._check(lambda x: len(x) > upperbound, 
                        "Length longer than %d" % upperbound)

        def each(self):
            subnames = []
            subvalues = []
            for name, value in zip(self.names, self.values):
                # Confirm each of these is a list, then break it down and populate the sub-Check 
                # object with the names and values of the individual items.
                self._check(lambda x: not isinstance(x, list), "Not a list")
                subnames += map(lambda index, subvalue: "%s[%s]" % (name, repr(index)), enumerate(value))
                subvalues += value
            return self.__class__(self.subjname, subnames, subvalues)

        def ascii(self):
            self._check(lambda x: not all(ord(c) < 128 for c in x), 
                        "String contains non-ascii characters.")
