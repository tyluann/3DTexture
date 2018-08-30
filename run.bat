Set exe=.\x64\Release\3DTexture.exe
Set data=.\dataset\rgbd_dataset_freiburg1_teddy
Set obj=%data%\mesh.obj
Set rgb=%data%\rgb_cam
Set out=%data%\out

md %out%
%exe% %obj% %rgb% %out% >%out%/log.txt
pause