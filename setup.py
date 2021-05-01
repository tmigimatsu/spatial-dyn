from setuptools import setup

import os
import re
import subprocess
import sys

from setuptools import Extension
from setuptools.command.build_ext import build_ext
from setuptools.command.install_lib import install_lib
from distutils.version import LooseVersion


class CMakeExtension(Extension):
    def __init__(self, name, sourcedir=""):
        Extension.__init__(self, name, sources=[])
        self.sourcedir = os.path.abspath(sourcedir)


class CMakeBuild(build_ext):
    def run(self):
        try:
            out = subprocess.check_output(["cmake", "--version"])
        except OSError:
            raise RuntimeError(
                "CMake must be installed to build the following extensions: "
                + ", ".join(e.name for e in self.extensions)
            )

        cmake_version = LooseVersion(
            re.search(r"version\s*([\d.]+)", out.decode()).group(1)
        )
        if cmake_version < "3.11.0":
            raise RuntimeError("CMake >= 3.11.0 is required.")

        for ext in self.extensions:
            self.build_extension(ext)

    def build_extension(self, ext):
        project_dir = os.path.dirname(os.path.realpath(__file__))
        build_dir = os.path.join(project_dir, "build")
        cmake_dir = os.path.join(project_dir, "cmake")
        try:
            os.mkdir(build_dir)
        except:
            pass
        ncpus = (
            subprocess.check_output(["./ncpu.sh"], cwd=cmake_dir)
            .strip()
            .decode("utf-8")
        )
        python_version = ".".join(map(str, sys.version_info[:3]))
        subprocess.check_call(
            [
                "cmake",
                "-DBUILD_TESTING=OFF",
                "-DPYBIND11_PYTHON_VERSION=" + python_version,
                "..",
            ],
            cwd=build_dir,
        )
        subprocess.check_call(
            ["cmake", "--build", ".", "--", "-j" + ncpus], cwd=build_dir
        )


class CMakeInstall(install_lib):
    def run(self):
        self.skip_build = True

        project_dir = os.path.dirname(os.path.realpath(__file__))
        lib_dir = os.path.join(project_dir, "lib", "spatialdyn")

        lib_files = [
            file for file in os.listdir(lib_dir) if os.path.splitext(file)[-1] == ".so"
        ]

        self.distribution.data_files = [
            (
                os.path.join(self.install_dir, "spatialdyn"),
                [os.path.join(lib_dir, file) for file in lib_files],
            )
        ]

        self.distribution.run_command("install_data")

        super().run()


setup(
    name="spatialdyn",
    version="1.4.0",
    description="Python wrapper for the C++ spatial_dyn library",
    url="https://github.com/tmigimatsu/spatial-dyn.git",
    author="Toki Migimatsu",
    license="MIT",
    ext_modules=[CMakeExtension("spatialdyn")],
    cmdclass={
        "build_ext": CMakeBuild,
        "install_lib": CMakeInstall,
    },
    packages=["spatialdyn"],
    package_dir={"": "lib"},
    install_requires=["numpy"],
)
