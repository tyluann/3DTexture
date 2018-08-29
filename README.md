# 3DTexture

This code is an implementation of "Let There Be Color! Large-Scale Texturing of 3D Reconstructions"(ECCV2014).

This project can only run on Windows platform. To do this, you just need to run "run.bat". This will run the example exe in ./x64/Release.

If you want to compile the code, There's a VS2017 project in it.

Directory structure:

./3Dtexture: VS project.

./dataset: test data. For now, there is only one set of data available. The mesh, rgb images and pose files were generated with one set of TUM datasets.

./sre: main source code of this project.

./thirdparty: open source libraries on which this project depends.
		- Eigen: for basic matrix calculation and solving linear equations.
		- Opencv: for storing images.
		- libacc: for 3D occlusion culling.
		- mapMAP: as a MRF solver (graph cut).
		- tbb: parallel computing, used by mapMAP.

./x64: example EXEs and DLLs.

./run.bat: one-key to run executable file.


