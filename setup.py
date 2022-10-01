from setuptools import setup

setup(
    name="ur5_gym",
    version="0.0.1",
    description="Set of OpenAI/gym robotic environments based on PyBullet physics engine for UR5 robot.",
    author="Yaroslav Savelev",
    author_email="yar21sav@gmail.com",
    packages=["ur5_gym"],
    license="GNU Lesser General Public License v3",
    install_requires=["gym>=0.21", "pybullet", "numpy", "opencv-python", "stable-baselines3", "tensorboard", "matplotlib", "urx"],
    classifiers=[
        "Programming Language :: Python",
        "Programming Language :: Python :: 3",
        "Development Status :: 4 - Beta",
        "Intended Audience :: Developers",
        "Operating System :: OS Independent",
        "Topic :: System :: Hardware :: Hardware Drivers",
        "Topic :: Software Development :: Libraries :: Python Modules",
    ])
