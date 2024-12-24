from setuptools import setup, find_packages

setup(
    name="mujoco_tools",
    version="0.1.0",
    packages=find_packages(),
    install_requires=[
        "mujoco>=3.2.0",
        "numpy>=1.23.0",
        "matplotlib>=3.4.0",
        "opencv-python>=4.5.0",
        "imageio>=2.9.0",
        "tqdm>=4.62.0",
    ],
    entry_points={
        'console_scripts': [
            'mujoco-tools=mujoco_tools.cli:main',
        ],
    },
    author="Shanning Zhuang",
    author_email="shanning.zhuang@outlook.com",
    description="A toolkit for MuJoCo simulation and visualization",
    long_description=open('README.md').read(),
    long_description_content_type="text/markdown",
    url="https://github.com/shanningzhuang/mujoco_tools",
    classifiers=[
        "Programming Language :: Python :: 3",
        "License :: OSI Approved :: MIT License",
        "Operating System :: OS Independent",
    ],
    python_requires='>=3.7',
) 