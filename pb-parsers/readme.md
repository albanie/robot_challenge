PB Parsers for CDT
==================
Matlab parsers for data obtained over MOOS from a Husky robot. Parsers are included for camera, 2D laser, and odometry data.

Contents
--------
- example.m: example file showing how to use the parsers to obtain data from MOOS
- mrg: MATLAB parsers for required data types
- lib: protocol buffer definitions for messages sent over MOOS
- internal: implementation details - should not be used directly

Usage
-----
Simply include the required code in MATLAB:

    javaaddpath('/path/to/pb-parsers/lib/protobuf-java.jar')
    javaaddpath('/path/to/pb-parsers/lib/datatypes_java.jar')
    addpath(genpath('/path/to/pb-parsers'))

See example.m for example usage of the included functions.
