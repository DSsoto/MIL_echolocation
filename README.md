# MIL_echolocation
Locates the position and time of emmission of a sound source

## Compiling and running
To compile:

	git clone https://github.com/DSsoto/MIL_echolocation.git
	cd MIL_echolocation
	mkdir build
	cd build
	cmake ..
	make
	./echolocation -h

The last command, will show proper usage, as of now, it can be called from the terminal with 4 timestamps.
Note: you will need to have ceres solver installed, see http://ceres-solver.org/building.html

## TODO
The positions of the hydrophones need to be replaced with the actual sensor locations in hydrophone_locate_source.cpp.