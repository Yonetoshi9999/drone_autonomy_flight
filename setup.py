from setuptools import setup, find_packages

setup(
    name="drone_gym",
    version="0.1.0",
    description="OpenAI Gym environment for autonomous drone simulation",
    author="Autonomous Drone Project",
    python_requires=">=3.8",
    packages=find_packages(),
    install_requires=[
        "numpy>=1.21.0",
        "scipy>=1.7.0",
        "gym>=0.26.0",
        "pymavlink>=2.4.37",
        # "airsim>=1.8.1",  # Skipped - AirSim not installed
        "opencv-python>=4.7.0",
        "matplotlib>=3.5.0",
        "pyyaml>=6.0",
    ],
    extras_require={
        "dev": [
            "pytest>=7.0.0",
            "black>=22.0.0",
            "flake8>=4.0.0",
        ],
        "rl": [
            "stable-baselines3[extra]>=2.0.0",
            "tensorboard>=2.11.0",
        ],
    },
    classifiers=[
        "Development Status :: 3 - Alpha",
        "Intended Audience :: Developers",
        "Intended Audience :: Science/Research",
        "License :: OSI Approved :: MIT License",
        "Programming Language :: Python :: 3",
        "Programming Language :: Python :: 3.8",
        "Programming Language :: Python :: 3.9",
        "Programming Language :: Python :: 3.10",
    ],
)
