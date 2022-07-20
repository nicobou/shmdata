# This program is free software; you can redistribute it and/or
# modify it under the terms of the GNU Lesser General Public License
# as published by the Free Software Foundation; either version 2.1
# of the License, or (at your option) any later version.

# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU Lesser General Public License for more details.

import typing


class PropsDescriptor:
    def __init__(self, owner, defaults: typing.Optional[typing.Dict] = None):
        if not isinstance(defaults, dict):
            raise TypeError(f"argument `defaults` is not a dictionary, received `{type(defaults).__name__}`")
        self.__data = defaults.copy() if defaults is not None else {}
        self.__owner = owner

    @property
    def __dict__(self):
        return self.__data.copy()

    def __get__(self, obj, owner=None):
        return self

    def __repr__(self):
        return str(self.__data)

    def __iter__(self):
        yield from self.__data

    def __getitem__(self, key):
        # check value was not modified since last access
        value = self.__owner.get(key)
        if self.__data[key] != value:
            self.__data[key] = value
        return self.__data[key]

    def __setitem__(self, key, value):
        if key in self.__data and self.__owner.set(key, value):
            self.__data[key] = self.__owner.get(key)

    def items(self):
        return self.__data.items()

    def update(self, data):
        self.__owner.update(data)
