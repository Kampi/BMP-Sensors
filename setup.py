import setuptools

with open("README.md", "r") as Help:
    long_description = Help.read()

setuptools.setup(
    name="bmp-sensors", 
    version="1.0.3",
    author="Daniel Kampert",
    author_email="DanielKampert@kampis-elektroecke.de",
    description="I2C driver for the Bosch Sensortec BMP-Sensors family.",
    long_description=long_description,
    long_description_content_type="text/markdown",
    url="https://gitlab.com/Kampi/bmp-sensors",
    packages=setuptools.find_packages(),
    classifiers=[
        "Programming Language :: Python :: 3",
        "License :: OSI Approved :: GNU General Public License v3 (GPLv3)",
        "Operating System :: OS Independent",
    ],
    python_requires='>=3.0',
	install_requires=["smbus"],
)