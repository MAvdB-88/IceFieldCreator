
# IceFieldCreator 

IceFieldCreator finds a non-overlapping assembly of randomly placed convex polygons in a rectangular domain.
It can be used to create random numerical broken ice fields for the simulation of sea ice-structure interaction.

# Dependencies

IceFieldCreator uses the following libraries:
- RapidJSON v1.1.0 (rapidjson.org)
- OpenCV v4.5.1 (opencv.org)
- STL


# Building

- Download and install/build OpenCV following the instructions.
- Download and unpack RapidJSON library (header-only)
- Install CMake.
- Update CMakeLists to point to the right include directories for OpenCV and RapidJSON.
- Run CMake.

- Project is developed and tested using Visual Studio 2019, v16.8.2.
- Project uses C++14.

# Usage

Using the command line:
- Start by typing IceFieldCreator.exe <input_file_name> when in the right directory.

By opening executable:
- IceFieldCreator will ask for an input file. Provide the file path.

Several example input files are provided in the data folder.

A simple GUI will open, allowing the user to monitor the progress, 
help with resolving the overlaps by dragging bodies, and interactively change max displacement
and margin parameters to improve convergence.

Stop by closing the GUI. IceFieldCreator will write the final body assembly to output.txt


# Contributing

If you want to contribute, please consider working on one of the following topics:

- Dynamics: IceFieldCreator is currently only used for resolving overlaps, but can be expanded quite
easily to a simple physics engine by including dynamics.

- Any contribution that improves readability (while not compromising efficiency) or efficiency (while not compromising readability)

- Please try to keep the same coding style.







