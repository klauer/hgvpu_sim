HGVPU sim
---------

LCLS-II specific simulator for Animatics SmartMotors (class 5)

Usage
-----

As the simulator uses asyncio, it requires Python 3:

```bash
 $ python3 sim.py
```

In a separate terminal:
```bash
 $ make
 $ cd iocBoot/iocsmartSim
 $ ./st.cmd
```

(NOTE: update architecture to rhel6 - development was done on OSX)

The included IOC is configured to connect to the simulator (on localhost, port 7001).
It allows for single motor mode and full coordinated motion mode.
