Set exe=.\x64\Release\3DTexture.exe
Set obj=.\dataset\rgbd_dataset_freiburg1_teddy\mesh.obj
Set rgb=.\dataset\rgbd_dataset_freiburg1_teddy\rgb_cam
Set out=.\dataset\rgbd_dataset_freiburg1_teddy\out

md %out%
%exe% %obj% %rgb% %out% >%out%/log.txt