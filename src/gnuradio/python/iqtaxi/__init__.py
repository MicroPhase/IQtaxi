from __future__ import absolute_import
from __future__ import unicode_literals

from gnuradio import gr  # noqa: F401

try:
    from .iqtaxi_python import *
except ModuleNotFoundError:
    try:
        from .iqtaxi_swig import *
    except ModuleNotFoundError:
        from iqtaxi_swig import *
