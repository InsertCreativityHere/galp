
# Purpose

This document describes and documents the communication protocol used between sensor interfaces (which transmit information from a sensor), and clients (which receive and request information from sensors/interfaces). This protocol makes no attempt to account for or correct errors in transmission and assumes the underlying transport mechanism to be stable and reliable in of itself. Instead this protocol is optimized for effeciency of transport and to minimize the amount of data sent over the wire.

It is purpose designed for use with GALP and the Vernier Sensor Interfaces and is likely unsuitable for most external applications.


# Data Encoding
It is presumed there are at maximum 



000
100
010
001
101