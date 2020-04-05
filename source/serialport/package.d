/++
    Crossplatform work with serialport.

    See also `example/monitor`
 +/
module serialport;

version (Posix) {} else version (Windows) {}
else static assert(0, "unsupported platform");

public:

import serialport.base;
import serialport.config;
import serialport.block;
import serialport.fiberready;
import serialport.evloopready;
import serialport.nonblock;
import serialport.exception;
import serialport.types;