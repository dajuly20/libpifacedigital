libpifacedigitalCpp
================

This is a approach for an C++ adoption of [libpifacedigital](https://github.com/piface/pifacedigital) by Julian Wiche  
The controll interface which is included in the original (pifacedigital-cmd.c)  is NOT included.

A simple C++ library for controlling
[PiFace Digital](http://www.piface.org.uk/products/piface_digital/).
Hides the SPI file descriptors so the user only has to deal with `hw_addr`.


Building
--------

To build the library, first install the required dependency
[libmcp23s17](https://github.com/piface/libmcp23s17). 

    $ cd  ~
    $ git clone https://github.com/piface/libmcp23s17
    $ cd  libmcp23s17
    $ make 

Install the library to `/usr/local` using: (optional)

    $ make install

    
Then run:

    $ cd ~
    $ git clone https://github.com/dajuly20/libpifacedigitalcpp
    $ cd libpifacedigitalcpp
    $ make

Install the library to `/usr/local` using: (optional)

    $ make install

To test the library, compile and execute the example program:

    $ make example
    $ ./example

Usage
-----

See `example.cpp` for example usage.

When you have made install, just compile your software with the following flags:

    -lpifacedigitalcpp -lmcp23s17

If not, you must specify the path of the libraries manually, using:

    -I/path/to/headers -L/path/to/libpifacedigital -lpifacedigitalcpp -L/path/to/mcp23s17 -lmcp23s17

Documentation
-------------

An online version of the documentation is available at http://piface.github.io/libpifacedigital.

Build it with (assuming that you are in the directory of the cloned repository):

    $ cd docs/
    $ doxygen pifacedigital-doc.conf

To view as HTML, point your browser to `docs/html/index.html`.

