from .spatialdyn import *
from .opspace_control import opspace_control
from .joint_space_control import joint_space_control

from ctrlutils import eigen

__version__ = "1.4.1"

eigen.Isometry3d.fdot = lambda self, f: fdot(self, f)
eigen.Isometry3d.mdot = lambda self, m: mdot(self, m)
eigen.Translation3d.fdot = lambda self, f: fdot(self, f)
eigen.Translation3d.mdot = lambda self, m: mdot(self, m)
