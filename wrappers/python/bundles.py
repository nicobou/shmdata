# This program is free software; you can redistribute it and/or
# modify it under the terms of the GNU Lesser General Public License
# as published by the Free Software Foundation; either version 2.1
# of the License, or (at your option) any later version.

# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU Lesser General Public License for more details.

import json
from dataclasses import dataclass


@dataclass
class Bundle:
    name: str
    pipeline: str
    doc: dict

    def __repr__(self):
        return f"<Bundle '{self.name}'>"

    def __str__(self):
        return json.dumps({
            self.name: {
                "pipeline": self.pipeline,
                "doc": self.doc
            }
        })
