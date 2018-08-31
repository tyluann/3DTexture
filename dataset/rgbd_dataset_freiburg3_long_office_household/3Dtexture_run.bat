Set exe=..\..\x64\Release\3DTexture.exe
Set data=.
Set obj=%data%\mesh.obj
Set rgb=%data%\rgb_cam
Set out=%data%\out

md %out%
%exe% %obj% %rgb% %out% >%out%/log.txt
pause