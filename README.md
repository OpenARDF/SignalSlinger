# SignalSlinger

SignalSlinger is an open-source 80-meter radio orienteering (ARDF) transmitter kit for the 3.5 MHz to 3.7 MHz amateur band. It supports classic, sprint, and foxoring events, with a high-accuracy real-time clock for synchronized transmissions and scheduled start and finish times.

## Product Photo

![SignalSlinger finished product](images/signal-slinger-finished-product.jpg)

## Documentation

* [User Manual](https://docs.google.com/document/d/1eX7xH3cDyRNS-MVg13EojpP8IB8bDgrdszM48J0sn7k/edit?usp=sharing)
* [Bill of Material](https://docs.google.com/spreadsheets/d/182rCsEmR_KNoESYd0NLeVXOi867AKD0zTovbhvcYbqc/edit?usp=sharing)
* [GitHub Releases](https://github.com/OpenARDF/SignalSlinger/releases)

## Updating Software

This `Development2` branch corresponds to the current development prerelease firmware. Download the release file that matches your hardware revision:

* `SignalSlinger-v1.2p-3.5.hex`
* `SignalSlinger-v1.2p-3.4.hex`

Use an Atmel-ICE programmer over the board's UPDI programming header (`P101`) to install the firmware. Always choose the `.hex` file that matches your hardware revision, and do not modify fuses or disable UPDI.

For complete programming steps, avrdude examples, and hardware connection details, see the [User Manual](https://docs.google.com/document/d/1eX7xH3cDyRNS-MVg13EojpP8IB8bDgrdszM48J0sn7k/edit?usp=sharing). If you want the stable public release instead of the development prerelease, switch to the `main` branch and use the release files referenced there.

## Related Projects

* [SignalStreamer](https://github.com/OpenARDF/SignalStreamer), a matching antenna
* [SignalSnagger](https://github.com/OpenARDF/SignalSnagger), a companion receiver project
