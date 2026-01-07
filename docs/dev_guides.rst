Developer Guide
===============

.. toctree::
   :maxdepth: 2


.. note::

   TODO: Short bit on asyncio and its traps


Navigation function guide
-------------------------

TODO: Explanation of the navigation system and how the components interact

Creating your own plugin
------------------------

TODO: Example process of making a plugin, using existing as example

Creating your own mission
-------------------------

TODO: Example process of making a mission, using existing as example

Example missions
----------------

TODO: Explanation of UAM, redcross, engel

Writing documentation
---------------------

We use Google-style docstrings. Most IDEs can be set up to configure
which style of docstring stub is generated.
Type hints go into the signature, not the docstring.

For classes, everything goes into the class docstring, except the arguments
for __init__, which go into the __init__ docstring.

Sphinx uses the class hierarchy to try and find a docstring when a class
overrides a member of its parent class. This can lead to errors when the
docstring of the parent class doesn't meet the formatting standards. In
this case, the subclass should provide its own docstring, referencing the
parent class when necessary.

For general formatting, see the sphinx documentation, they have examples
of Google-style docstrings as well.
