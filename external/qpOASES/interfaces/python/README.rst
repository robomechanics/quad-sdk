pyqpOASES: a Python interface to qpOASES

:Author: Sebastian F. Walter, Manuel Kudruss

Known to work with
------------------

* python2.7
* qpOASES 3.2 (rev 259)


Installation
------------

Requirements:

  You'll need numpy and cython. Install for instance with::

      sudo pip install cython
      sudo pip install numpy

Method 1:

  This is a local installation and creates `./interfaces/python/qpoases.so`::

    make python

  Then, you'll have to update your PYTHONPATH, e.g., on LINUX you have to add::

    export PYTHONPATH=$PYTHONPATH:/home/swalter/projects/qpOASES/interfaces/python

  to your ``~/.bashrc``.

Method 2:

  global installation::

    sudo make pythoninstall

Method 3::

   cd ./interfaces/python/
   python setup.py build_ext --inplace
   # or python setup.py install


Testing your installation
-------------------------

For a quick test run::

  cd ./interfaces/python
  python example1.py


To run a complete unit test you need ``nose``. Install for instance with::

    sudo pip install nose

Then::

   cd ./interfaces/python/
   nosestests ./tests

The results of the tests can be found in `./interfaces/python/tests/results`.

Tested setups
-------------

The Python interface is known to work on

* Windows, Python 3
* Linux (Ubuntu 12.04) using Python 2.7.3, Python 3.2.3. NumPy 1.8, Cython 0.19
* MacOS Mojave (10.14.3) using Python 3.7 (anaconda suite)

