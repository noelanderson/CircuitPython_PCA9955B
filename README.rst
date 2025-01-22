Introduction
============


.. image:: https://readthedocs.org/projects/circuitpython-pca9955b/badge/?version=latest
    :target: https://circuitpython-pca9955b.readthedocs.io/
    :alt: Documentation Status



.. image:: https://img.shields.io/discord/327254708534116352.svg
    :target: https://adafru.it/discord
    :alt: Discord


.. image:: https://github.com/noelanderson/CircuitPython_PCA9955B/workflows/Build%20CI/badge.svg
    :target: https://github.com/noelanderson/CircuitPython_PCA9955B/actions
    :alt: Build Status


.. image:: https://img.shields.io/endpoint?url=https://raw.githubusercontent.com/astral-sh/ruff/main/assets/badge/v2.json
    :target: https://github.com/astral-sh/ruff
    :alt: Code Style: Ruff

CircuitPython helper library for the NXP 16-Channel I²C-Bus Constant-Current LED Driver


Dependencies
=============
This driver depends on:

* `Adafruit CircuitPython <https://github.com/adafruit/circuitpython>`_
* `Bus Device <https://github.com/adafruit/Adafruit_CircuitPython_BusDevice>`_

Please ensure all dependencies are available on the CircuitPython filesystem.
This is easily achieved by downloading
`the Adafruit library and driver bundle <https://circuitpython.org/libraries>`_
or individual libraries can be installed using
`circup <https://github.com/adafruit/circup>`_.

Installing from PyPI
=====================
.. note:: This library is not available on PyPI yet. Install documentation is included
   as a standard element. Stay tuned for PyPI availability!


On supported GNU/Linux systems like the Raspberry Pi, you can install the driver locally `from
PyPI <https://pypi.org/project/circuitpython-pca9955b/>`_.
To install for current user:

.. code-block:: shell

    pip3 install circuitpython-pca9955b

To install system-wide (this may be required in some cases):

.. code-block:: shell

    sudo pip3 install circuitpython-pca9955b

To install in a virtual environment in your current project:

.. code-block:: shell

    mkdir project-name && cd project-name
    python3 -m venv .venv
    source .env/bin/activate
    pip3 install circuitpython-pca9955b

Installing to a Connected CircuitPython Device with Circup
==========================================================

Make sure that you have ``circup`` installed in your Python environment.
Install it with the following command if necessary:

.. code-block:: shell

    pip3 install circup

With ``circup`` installed and your CircuitPython device connected use the
following command to install:

.. code-block:: shell

    circup install pca9955b

Or the following command to update an existing version:

.. code-block:: shell

    circup update

Usage Example
=============

.. code-block:: python

    import time
    import board
    import busio

    from pca9955b import PCA9955, LedChannel
    i2c = busio.I2C(board.SCL, board.SDA)

    ledDriver = PCA9955(i2c, address=0x3F, oe_pin=board.GP10, reset_pin=board.GP11)
    ledDriver.reset()
    ledDriver.output_enable = True

    ledDriver.gain = 0xFF
    ledDriver.brightness = 0x7F  # 50% brightness

    ledDriver.channels[0].output_state = LedChannel.ON
    time.sleep(5)
    ledDriver.channels[0].output_state = LedChannel.OFF
    time.sleep(5)

    ledDriver.channels[0].output_state = LedChannel.PWM
    for i in range(255):
        ledDriver.channels[0].brightness = i
        time.sleep(0.02)


Documentation
=============

Class Diagram for library

.. figure:: https://raw.githubusercontent.com/noelanderson/CircuitPython_PCA9955B/master/pca9955bClasses.svg
   :alt: Class Diagram

API documentation for this library can be found on `Read the Docs <https://circuitpython-pca9955b.readthedocs.io/>`_.


Contributing
============

Contributions are welcome! Please read our `Code of Conduct
<https://github.com/noelanderson/CircuitPython_PCA9955B/blob/HEAD/CODE_OF_CONDUCT.md>`_
before contributing to help this project stay welcoming.
