import pathlib
import setuptools
import subprocess

from setuptools.command import build_ext


class CMakeExtension(setuptools.Extension):
    def __init__(self, name):
        setuptools.Extension.__init__(self, name, sources=[])


class CMakeBuild(build_ext.build_ext):
    def run(self):
        import distutils
        import re

        if not self.inplace:
            try:
                out = subprocess.check_output(["cmake", "--version"])
            except OSError:
                raise RuntimeError(
                    "CMake must be installed to build the following extensions: "
                    + ", ".join(e.name for e in self.extensions)
                )

            cmake_version = distutils.version.LooseVersion(
                re.search(r"version\s*([\d.]+)", out.decode()).group(1)
            )
            if cmake_version < "3.13.0":
                raise RuntimeError(
                    "CMake >= 3.13.0 is required. Install the latest CMake with 'pip install cmake'."
                )

        for extension in self.extensions:
            self.build_extension(extension)

    def build_extension(self, extension: setuptools.Extension):
        import os
        import pathlib
        import re
        import shutil
        import sys

        extension_dir = pathlib.Path(self.get_ext_fullpath(extension.name)).parent
        extension_dir.mkdir(parents=True, exist_ok=True)

        if self.inplace:
            build_dir = extension_dir / "build"
        else:
            build_dir = pathlib.Path(self.build_temp)
        build_dir.mkdir(parents=True, exist_ok=True)

        # Run CMake.
        build_type = "Debug" if self.debug else "Release"
        python_version = ".".join(map(str, sys.version_info[:3]))
        cmake_command = [
            "cmake",
            "-B" + str(build_dir),
            "-DBUILD_TESTING=OFF",
            "-DPYBIND11_PYTHON_VERSION=" + python_version,
            "-DCMAKE_BUILD_TYPE=" + build_type,
        ]
        if not self.inplace:
            # Use relative paths for install rpath.
            rpath_origin = "@loader_path" if sys.platform == "darwin" else "$ORIGIN"
            cmake_command += [
                "-DCMAKE_INSTALL_PREFIX=install",
                "-DCMAKE_INSTALL_RPATH=" + rpath_origin,
            ]
        self.spawn(cmake_command)

        # Build and install.
        make_command = ["cmake", "--build", str(build_dir)]
        if not self.inplace:
            make_command += ["--target", "install"]

        ncpus = (
            subprocess.check_output(["./ncpu.sh"], cwd="cmake").strip().decode("utf-8")
        )
        make_command += ["--", "-j" + ncpus]

        self.spawn(make_command)

        if not self.inplace:
            # Copy pybind11 library.
            spatialdyn_dir = str(extension_dir / "spatialdyn")
            for file in os.listdir(build_dir / "src" / "python"):
                if re.match(r".*\.(?:so|dylib)\.?", file) is not None:
                    file = str(build_dir / "src" / "python" / file)
                    shutil.move(file, spatialdyn_dir)

            # Copy C++ libraries.
            for file in os.listdir(os.path.join("install", "lib")):
                if re.match(r".*\.(?:so|dylib)\.?", file) is not None:
                    file = os.path.join("install", "lib", file)
                    shutil.move(file, spatialdyn_dir)


setuptools.setup(
    name="spatialdyn",
    version="1.4.0",
    author="Toki Migimatsu",
    description="spatial-dyn library",
    url="https://github.com/tmigimatsu/spatial-dyn.git",
    license="MIT",
    packages=["spatialdyn"],
    classifiers=[
        "Programming Language :: Python :: 3",
        "License :: OSI Approved :: MIT License",
        "Operating System :: OS Independent",
    ],
    python_requires=">=3.6",
    ext_modules=[CMakeExtension("spatialdyn")],
    cmdclass={
        "build_ext": CMakeBuild,
    },
    install_requires=[
        "numpy",
        f"ctrlutils @ file://{pathlib.Path.cwd() / 'external' / 'ctrl_utils' / 'ctrl-utils.git'}",
    ],
)
