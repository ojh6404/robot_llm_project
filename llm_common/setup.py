from catkin_pkg.python_setup import generate_distutils_setup
from distutils.core import setup


d = generate_distutils_setup(
    packages=["llm_common"],
    package_dir={"": "src"},
    version="0.0.1",
    install_requires=["setuptools", "openai", "opencv-python", "flask", "requests"],
)

setup(**d)
