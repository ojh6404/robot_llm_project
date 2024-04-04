from catkin_pkg.python_setup import generate_distutils_setup
from setuptools import setup


d = generate_distutils_setup(
    packages=["llm_ros"],
    package_dir={"": "src"},
    version="0.0.1",
    install_requires=["setuptools", "openai", "opencv-python", "flask", "requests", "tiktoken", "openai"],
    # author='Jihoon Oh',
    # author_email='oh@jsk.imi.i.u-tokyo.ac.jp',
    # keywords=['ROS'],
    # classifiers=[
    #     'Intended Audience :: Developers',
    #     'License :: OSI Approved :: MIT License',
    #     'Programming Language :: Python',
    #     'Topic :: Software Development',
    # ],
    # description='A ROS1 package for Large Language Model.',
    # long_description=open('README.md').read(),
    # long_description_content_type='text/markdown',
    # license='MIT',
)

setup(**d)
