import os
import sys

from setuptools import find_packages, setup

def read(rel_path: str, encoding: str=None) -> str:
    here = os.path.abspath(os.path.dirname(__file__))
    if encoding is not None:
        with open(os.path.join(here, rel_path), encoding=encoding) as fp:
            return fp.read()
    else:
        with open(os.path.join(here, rel_path)) as fp:
            return fp.read()


def get_version(rel_path: str) -> str:
    for line in read(rel_path).splitlines():
        if line.startswith("__version__"):
            # __version__ = "0.9"
            delim = '"' if '"' in line else "'"
            return line.split(delim)[1]
    raise RuntimeError("Unable to find version string.")
    

long_description = read("README.md", encoding="utf-8")


setup(
    name="MMC5983MA",
    version=get_version("src/MMC5983MA/__init__.py"),
    author="Felipe Calliari",
    author_email="calliari@puc-rio.br",
    description="Python implementation for the MMC5983MA 3-axis magnetometer",
    long_description=long_description,
    long_description_content_type="text/markdown",
    url="https://github.com/FelipeCalliari/MMC5983MA",
    project_urls={
        "Bug Tracker": "https://github.com/FelipeCalliari/MMC5983MA/issues",
    },
    classifiers=[
        "Programming Language :: Python :: 3 :: Only",
        "Programming Language :: Python :: 3.6",
        "Programming Language :: Python :: 3.7",
        "Programming Language :: Python :: 3.8",
        "Programming Language :: Python :: 3.9",
        "Programming Language :: Python :: 3.10",
        "Programming Language :: Python :: 3.11",
        "License :: OSI Approved :: GNU Lesser General Public License v3 (LGPLv3)",
        "Operating System :: POSIX :: Linux",
        "Development Status :: 2 - Pre-Alpha",
        "Intended Audience :: Science/Research",
        "Topic :: Scientific/Engineering",
        "Topic :: Software Development :: Libraries :: Python Modules",
    ],
    package_dir={"": "src"},
    packages=find_packages(
        where="src",
        exclude=["docs", "tests*"],
    ),
    python_requires=">=3.6",
)
