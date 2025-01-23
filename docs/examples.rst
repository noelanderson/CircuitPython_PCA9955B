Simple test
------------

Ensure your device works with this simple test.

.. literalinclude:: ../examples/pca9955b_simpletest.py
    :caption: examples/pca9955b_simpletest.py
    :linenos:


Global Control (Blinking)
-------------------------

This example uses global controls to continuously blink lights without
having the processor to do anything.

.. literalinclude:: ../examples/pca9955b_global_control.py
    :caption: examples/pca9955b_global_control.py
    :linenos:


Group Control (Sawtooth pattern)
--------------------------------

This example uses group controls to have a set of channels continuously turn on,
brighten, and then turn off in a sawtoth patttern, without processor activity.

.. literalinclude:: ../examples/pca9955b_group_control.py
    :caption: examples/pca9955b_group_control.py
    :linenos:
