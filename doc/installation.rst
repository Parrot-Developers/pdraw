.. _installation:

Installation
============

System requirements
-------------------

*PDrAW* works as a standalone program & library for Linux
(tested on Ubuntu 18.04) and macOS (tested on macOS 10.14.4 Mojave)

You need to install the following packages:

Linux:

.. code-block:: console

    $ apt install python git repo python3 build-essential pkg-config zlib1g-dev libglfw3-dev libsdl2-dev rsync

macOS:

We use the homebrew_ package manager to install packages under macOS.
homebrew_ requires Xcode (or Xcode command line tools).
If you use another package manager, you will need
to install the required packages manually.

We also require the Xcode command line tools, with header package
installed in ``/``.

.. _homebrew: https://brew.sh/

.. code-block:: console

    # Install homebrew + Xcode first
    # Note: check the macOS version in the package name!
    $ installer -pkg /Library/Developer/CommandLineTools/Packages/macOS_SDK_headers_for_macOS_10.14.pkg -target /
    $ brew install repo gpg python3 glfw sdl2 pkg-config

Clone the GroundSDK workspace
-----------------------------

*PDrAW* is part of Parrot's *GroundSDK* workspace, so you need to clone that
workspace, using the ``repo`` tool.

.. code-block:: console

    # Assuming you want to put the code in $HOME/code/groundsdk
    $ mkdir -p $HOME/code/groundsdk
    $ cd $HOME/code/groundsdk
    $ repo init -u https://github.com/Parrot-Developers/groundsdk-manifest -m release.xml
    $ repo sync

After the initial clone, only the ``repo sync`` command will be needed to
update your workspace to the latest *GroundSDK* version.

Build PDrAW
-----------

After each update, the *PDrAW* project needs to be rebuilt with the following
command:

.. code-block:: console

    # Run from the workspace root directory
    $ ./build.sh -p pdraw-linux -t build -j/1

.. Note:: For macOS users, replace the following parts of example command lines:

   - ``native-wrapper.sh`` with ``native-darwin-wrapper.sh``
   - ``pdraw-linux`` with ``pdraw-macos``

.. Note:: This commands builds both the *pdraw* and *vmeta-extract*
   executables, and all the required libraries required to use *PDrAW* from your
   own project.

Run PDrAW
---------

In order to run the built binaries, you need to use a wrapper script provided
in the ``out`` directory:

.. code-block:: console

    # Run from the workspace root directory
    $ ./out/pdraw-linux/staging/native-wrapper.sh pdraw --help
