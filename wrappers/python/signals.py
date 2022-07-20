# This program is free software; you can redistribute it and/or
# modify it under the terms of the GNU Lesser General Public License
# as published by the Free Software Foundation; either version 2.1
# of the License, or (at your option) any later version.

# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU Lesser General Public License for more details.

import typing


class SignalsDescriptor:
    def __init__(self, defaults: typing.Optional[typing.Dict[str, typing.List[typing.Callable]]] = None):
        if defaults:
            defaults = defaults.copy()
        self.__data = defaults or {}

    def __get__(self, obj, owner=None):
        return self

    def __repr__(self):
        return str(self.__data)

    def __getitem__(self, key):
        return self.__data[key]

    def subscribe(self, key: str, cb: typing.Callable) -> bool:
        try:
            check = key in self.__data and cb not in self.__data[key]
            if check:
                self.__data[key].append(cb)
            return check
        except Exception as e:
            raise e

    def unsubscribe(self, key: str, idx: typing.Optional[int] = None) -> bool:
        if key in self.__data:
            try:
                if idx:
                    self.__data[key].pop(idx)
                else:
                    self.__data[key].clear()
                return True
            except Exception as e:
                raise e
        else:
            return False


class SignalRegistry:
    def __init__(self, defaults: typing.Optional[typing.List[typing.Callable]] = None):
        self.__data = defaults or []

    def __len__(self):
        return len(self.__data)

    def __iter__(self):
        yield from self.__data

    def __repr__(self):
        return str(self.__data)

    def append(self, cb: typing.Callable):
        if cb not in self.__data:
            self.__data.append(cb)

    def pop(self, idx: int):
        return self.__data.pop(idx)

    def index(self, o):
        return self.__data.index(o)

    def clear(self):
        self.__data.clear()
