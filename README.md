# PC_error_concealment

## Environment requirement
Testing Environment Version:

Ubuntu 20.04

Cmake 3.23.2 (https://cmake.org/install/)

Cpp point cloud library 1.10.0 (https://pointclouds.org/downloads/),

Example install command:
```bach
sudo apt update
sudo apt install cmake g++ make
sudo apt install libpcl-dev
```

## How to build the project using cmake
Compile executable file `Pipeline` under a new folder `build`
```bach
git clone git@github.com:Hunag-I-Chun/error_concealment_pipeline.git
cd error_concealment_pipeline
mkdir build
cd build
cmake ..
make
```

## Usage
Go back to folder `error_concealment_pipeline`
```bash
cd ..
```

Run following command:
```bash
python client.py -prev PREVIOUS_FRAME_PATH -next NEXT_FRAME_PATH -out OUTPUT_PATH -pos RELATIVE_POSITION
``` 

Example command:
```bash
python client.py -prev files/longdress_1051.ply -next files/longdress_1055.ply -out output.ply -pos 0.5
```
`-prev files/longdress_1051.ply` previous point cloud file we provide

`-next files/longdress_1055.ply` next point cloud file we provide

`-out files/output.ply` output point cloud file path

`-pos 0.5` means the output file is right at the middle of previous and next frames

The output.ply will look like:
![output.ply](files/output_illustration.png)

## Different pipeline sample
In `error_concealment_pipeline/config.hpp`, we've provided P, B, Q pipeline combinations for your convenience. The current pipeline in use is labeled as "Q." To switch to a different pipeline combination, simply comment or uncomment the relevant lines based on your requirements. Remember that every time you make changes to either the .hpp or .cpp files, you'll need to rebuild the project.

## How to create your own combination/stage method
To create custom methods, please consult the `error_concealment_pipeline/base.hpp` file. Within `base.hpp`, we've established a virtual class called Stage, which all methods must inherit from. Additionally, in the `Pipeline_Object` definition, we've clearly specified the output variables for each stage. It's essential to ensure that any custom stage methods you create align with the input and output requirements of each respective stage. After implement your own stage method, modify the pipeline reference in `error_concealment_pipeline/config.hpp`, and rebuild the project.

## Other tips
Given that various point cloud datasets may employ different coordinate systems and units of measurement, we recommend scaling your dataset to align with the MPEG (http://plenodb.jpeg.org/pc/8ilabs/) for consistency.
