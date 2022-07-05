import pathlib
import re
import shutil
import setuptools  # type: ignore
from setuptools.command import build_ext  # type: ignore
import subprocess
import sys


__version__ = "1.4.2"


class CMakeExtension(setuptools.Extension):
    def __init__(self, name):
        setuptools.Extension.__init__(self, name, sources=[])


class CMakeBuild(build_ext.build_ext):
    def run(self):
        from packaging import version  # type: ignore

        if not self.inplace:
            try:
                out = subprocess.check_output(["cmake", "--version"])
            except OSError:
                raise RuntimeError(
                    "CMake must be installed to build the following extensions: "
                    + ", ".join(e.name for e in self.extensions)
                )

            cmake_version = version.Version(
                re.search(r"version\s*([\d.]+)", out.decode()).group(1)
            )
            if cmake_version < version.Version("3.13.0"):
                raise RuntimeError(
                    "CMake >= 3.13.0 is required. Install the latest CMake with 'pip install cmake'."
                )

        for extension in self.extensions:
            self.build_extension(extension)

    def build_extension(self, extension: setuptools.Extension):
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
            "-DBUILD_EXAMPLES=OFF",
            "-DBUILD_PYTHON=ON",
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
            for file in (build_dir / "src" / "python").iterdir():
                if re.match(r".*\.(?:so|dylib)\.?", file.name) is not None:
                    shutil.move(str(file), spatialdyn_dir)

            # Copy C++ libraries.
            libdir = next(iter(pathlib.Path("install").glob("lib*")))
            for file in libdir.iterdir():
                if re.match(r".*\.(?:so|dylib)\.?", file.name) is not None:
                    shutil.move(str(file), spatialdyn_dir)


setuptools.setup(
    name="spatialdyn",
    version=__version__,
    author="Toki Migimatsu",
    description="spatial-dyn library",
    long_description="spatial-dyn is a C++/Python rigid body dynamics library.",
    long_description_content_type="text/plain",
    url="https://github.com/tmigimatsu/spatial-dyn.git",
    license="MIT",
    packages=["spatialdyn"],
    classifiers=[
        "Programming Language :: Python :: 3",
        "License :: OSI Approved :: MIT License",
        "Operating System :: OS Independent",
    ],
    python_requires=">=3.6",
    setup_requires=["packaging"],
    ext_modules=[CMakeExtension("spatialdyn")],
    cmdclass={
        "build_ext": CMakeBuild,
    },
    install_requires=["numpy", "ctrlutils"],
)
