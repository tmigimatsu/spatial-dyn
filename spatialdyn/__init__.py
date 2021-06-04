from .spatialdyn import *
from .opspace_control import opspace_control

from ctrlutils import eigen

eigen.Isometry3d.fdot = lambda self, f: fdot(self, f)
eigen.Isometry3d.mdot = lambda self, m: mdot(self, m)
eigen.Translation3d.fdot = lambda self, f: fdot(self, f)
eigen.Translation3d.mdot = lambda self, m: mdot(self, m)
