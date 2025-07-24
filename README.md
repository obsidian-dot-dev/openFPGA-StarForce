# openFPGA-Starforce

This is a port of the [MiSTer-StarForce](https://github.com/madoov/MiSTer-StarForce) StarForce-compatible gateware core by madoov for the Analogue Pocket.

The core currently supports the following games:
- StarForce
- Baluba-louk no Densetsu

## Controls

Controls are mapped as follows on the Analogue Pocket:

| Function | Keys |
|--|--|
| Ship Movement |  D-Pad |
| Fire/Jump | A-button |
| Insert Coin: | Select (-) |
| Start | Start (+) |

Additional button mapping is supported via the "input" menu.

## Interact Menu

The core provides an "interact" menu, allowing a user to adjust DIP switches and other
hardware controls.

DIP Switches:

- Starting lives
- Bonus live scores
- Difficulty level 
- Enable or Disable Demo Sounds + Demo Music

DIP switch changes take effect after resetting the core.

Game reset can also be controlled from the interact menu from the Analogue Pocket OSD.

## Version History

* v0.9.0
- Initial Beta release

## Notes

This core is in beta; please flag any issues to this github.

## Known Limitations/Issues

Top/Bottom scanlines are cropped.

## License

Source code for the openFPGA integration of this core is provided under the
terms of GPLv3. Please see the component repositories for licensing details
of individual modules. 

## Attribution

All credits for the Starforce-compatible FPGA core are due the original authors
and contributors, without which this port would not have been possible.

## Installation

Copy the contents of the "dist" folder to the root of the SD card, along with the converted ROM file as described below.

### ROM Instructions

ROM files are *not included*, you must use [mra-tools-c](https://github.com/sebdel/mra-tools-c/)
along with the provided `*.mra` files to convert a MAME-compatible romsets to singular
`*.rom` files compatible with this core. These ROM files should be placed under:
`/Assets/starforce/common`.
