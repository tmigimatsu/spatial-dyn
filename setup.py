from setuptools import setup

import os
import re
import subprocess

from setuptools import Extension
from setuptools.command.build_ext import build_ext
from distutils.version import LooseVersion


class CMakeExtension(Extension):
    def __init__(self, name, sourcedir=''):
        Extension.__init__(self, name, sources=[])
        self.sourcedir = os.path.abspath(sourcedir)


class CMakeBuild(build_ext):
    def run(self):
        try:
            out = subprocess.check_output(['cmake', '--version'])
        except OSError:
            raise RuntimeError("CMake must be installed to build the following extensions: " +
                               ", ".join(e.name for e in self.extensions))

        cmake_version = LooseVersion(re.search(r'version\s*([\d.]+)', out.decode()).group(1))
        if cmake_version < '3.6.0':
            raise RuntimeError("CMake >= 3.6.0 is required.")

        for ext in self.extensions:
            self.build_extension(ext)

    def build_extension(self, ext):
        project_dir = os.path.dirname(os.path.realpath(__file__))
        build_dir = os.path.join(project_dir, 'build')
        cmake_dir = os.path.join(project_dir, 'cmake')
        try:
            os.mkdir(build_dir)
        except:
            pass
        ncpus = subprocess.check_output(['./ncpu.sh'], cwd=cmake_dir).strip().decode('utf-8')
        subprocess.check_call(['cmake', '..'], cwd=build_dir)
        subprocess.check_call(['cmake', '--build', '.', '--', '-j' + ncpus], cwd=build_dir)

setup(
    name="spatialdyn",
    version="1.0.0",
    description="Python wrapper for the C++ SpatialDyn library",
    url="https://github.com/tmigimatsu/spatial-dyn.git",
    author="Toki Migimatsu",
    license="MIT",
    ext_modules=[CMakeExtension('spatialdyn')],
    cmdclass=dict(build_ext=CMakeBuild)
)
