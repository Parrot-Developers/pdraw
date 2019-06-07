.. _userguide:

User Guide
==========

Use the PDrAW program
---------------------

The *PDrAW* program can be used to display a live stream from an *Anafi*,
connected directly with wifi or through a *SkyController*.

.. Note:: For macOS users, replace the following parts of example command lines:

   - ``groundsdk-linux`` with ``groundsdk-macos``

.. code-block:: console

    # Run from the workspace root directory
    $ ./out/groundsdk-linux/staging/native-wrapper.sh pdraw -u rtsp://192.168.42.1/live
    # Or rtsp://192.168.53.1/live if connected through a SkyController

It can also be used to replay a file from the drone memory/SD-card

.. code-block:: console

    # Run from the workspace root directory
    $ ./out/groundsdk-linux/staging/native-wrapper.sh pdraw -u rtsp://192.168.42.1/replay/100000010001.MP4
    # Or rtsp://192.168.53.1/replay/XXXX if connected through a SkyController
    # The replay URL can be queried from the drone webserver:
    # http://anafi.local for Anafi or http://anafi-ai.local for Anafi Ai

Or play a local file

.. code-block:: console

    # Run from the workspace root directory
    $ ./out/groundsdk-linux/staging/native-wrapper.sh pdraw -u path/to/file.mp4
    # No need to be connected to a drone!

Further options can be found with the ``-h`` flag, e.g.:

- ``--hud`` adds an overlayed display on the video, computed from the metadata
- ``--zebras`` enables overexposure zebra computation

Build your own app with libpdraw-backend
----------------------------------------

To build an application using *libpdraw-backend*, you can either
`use alchemy`_ to build your app, or use `your own build system`_.

.. Note:: The same applies when using *libpdraw* directly, but your app will
   be responsible for providing and running the *pomp_loop* event loop.

.. _use alchemy:

Using Alchemy to create a new executable
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

*Alchemy* searches for modules in the ``packages`` directory of your SDK
workspace. For this example, we will create an application in the
``<SDK>/packages/test-app`` directory.

First, create an ``atom.mk`` file in this directory. This file tells *Alchemy*
what to build, and which dependencies are required:

.. literalinclude:: examples/atom.mk.sample
    :language: makefile

Then create a ``main.c`` file. The provided example creates a *pdraw-backend*
instance, opens the given url (or local path), then closes the instance.
It implements the very basic operation of *libpdraw-backend*, and is sufficient
to check if the binary is properly linked.

.. _main:

.. literalinclude:: examples/main.c
    :language: C
    :linenos:

The application can then be built with the following command:

.. code-block:: console

    # Run from the workspace root directory
    $ ./build.sh -p groundsdk-linux -A test-app -j/1

And run with the following command:

.. code-block:: console

    # Run from the workspace root directory
    $ ./out/groundsdk-linux/staging/native-wrapper.sh test-app rtsp://192.168.42.1/live

.. _your own build system:

Using a custom build system with the SDK libraries
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

If you don't want to use *Alchemy* to build your app, you can directly use the
output files from the *Alchemy* build (libraries and headers) from any other
build system.

First, we need to build the SDK from *Alchemy* to be used externally:

.. code-block:: console

    # Run from the workspace root directory
    $ ./build.sh -p groundsdk-linux -A sdk -j/1

For this example, we will use the same main_ file from the *Alchemy* sample,
and assume that the SDK was properly built from a folder called ``<SDK>``.
The following makefile can be used to build the same binary as before, but
outside of the *Alchemy* build system:

.. literalinclude:: examples/Makefile
    :language: makefile
    :emphasize-lines: 5-6

As the libraries from *Alchemy* are not installed in the system directories,
you have to provide the library path to the loader when running the result
binary:

.. code-block:: console

    # Run from your Makefile/main.c directory
    $ make
    $ env LD_LIBRARY_PATH=<SDK>/out/groundsdk-linux/sdk/usr/lib ./test-app rtsp://192.168.42.1/live
