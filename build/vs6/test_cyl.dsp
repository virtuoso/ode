# Microsoft Developer Studio Project File - Name="test_cyl" - Package Owner=<4>
# Microsoft Developer Studio Generated Build File, Format Version 6.00
# ** DO NOT EDIT **

# TARGTYPE "Win32 (x86) Console Application" 0x0103

CFG=test_cyl - Win32 DebugDLL
!MESSAGE This is not a valid makefile. To build this project using NMAKE,
!MESSAGE use the Export Makefile command and run
!MESSAGE 
!MESSAGE NMAKE /f "test_cyl.mak".
!MESSAGE 
!MESSAGE You can specify a configuration when running NMAKE
!MESSAGE by defining the macro CFG on the command line. For example:
!MESSAGE 
!MESSAGE NMAKE /f "test_cyl.mak" CFG="test_cyl - Win32 DebugDLL"
!MESSAGE 
!MESSAGE Possible choices for configuration are:
!MESSAGE 
!MESSAGE "test_cyl - Win32 ReleaseLib" (based on "Win32 (x86) Console Application")
!MESSAGE "test_cyl - Win32 DebugLib" (based on "Win32 (x86) Console Application")
!MESSAGE "test_cyl - Win32 ReleaseDLL" (based on "Win32 (x86) Console Application")
!MESSAGE "test_cyl - Win32 DebugDLL" (based on "Win32 (x86) Console Application")
!MESSAGE 

# Begin Project
# PROP AllowPerConfigDependencies 0
# PROP Scc_ProjName ""
# PROP Scc_LocalPath ""
CPP=cl.exe
MTL=midl.exe
RSC=rc.exe

!IF  "$(CFG)" == "test_cyl - Win32 ReleaseLib"

# PROP BASE Use_MFC 0
# PROP BASE Use_Debug_Libraries 1
# PROP BASE Output_Dir "../../lib/ReleaseLib"
# PROP BASE Intermediate_Dir "obj/cyl/ReleaseLib"
# PROP BASE Target_Dir ""
# PROP Use_MFC 0
# PROP Use_Debug_Libraries 1
# PROP Output_Dir "../../lib/ReleaseLib"
# PROP Intermediate_Dir "obj/cyl/ReleaseLib"
# PROP Target_Dir ""
# ADD BASE CPP /nologo /MDd /W3 /Gm /GR /GX /ZI /Od /I "../../include" /D "_CRT_SECURE_NO_DEPRECATE" /D "WIN32" /YX /FD /GZ /c
# ADD CPP /nologo /MDd /W3 /Gm /GR /GX /ZI /Od /I "../../include" /D "_CRT_SECURE_NO_DEPRECATE" /D "WIN32" /YX /FD /GZ /c
# ADD BASE RSC /l 0x409 /d "_DEBUG"
# ADD RSC /l 0x409 /d "_DEBUG"
BSC32=bscmake.exe
# ADD BASE BSC32 /nologo
# ADD BSC32 /nologo
LINK32=link.exe
# ADD BASE LINK32 user32.lib gdi32.lib opengl32.lib glu32.lib /nologo /entry:"mainCRTStartup" /subsystem:console /incremental:yes /debug /machine:I386 /out:"../../lib/ReleaseLib/test_cyl.exe" /pdbtype:sept /libpath:"../../lib/ReleaseLib"
# ADD LINK32 user32.lib gdi32.lib opengl32.lib glu32.lib /nologo /entry:"mainCRTStartup" /subsystem:console /incremental:yes /debug /machine:I386 /out:"../../lib/ReleaseLib/test_cyl.exe" /pdbtype:sept /libpath:"../../lib/ReleaseLib"

!ELSEIF  "$(CFG)" == "test_cyl - Win32 DebugLib"

# PROP BASE Use_MFC 0
# PROP BASE Use_Debug_Libraries 1
# PROP BASE Output_Dir "../../lib/DebugLib"
# PROP BASE Intermediate_Dir "obj/cyl/DebugLib"
# PROP BASE Target_Dir ""
# PROP Use_MFC 0
# PROP Use_Debug_Libraries 1
# PROP Output_Dir "../../lib/DebugLib"
# PROP Intermediate_Dir "obj/cyl/DebugLib"
# PROP Target_Dir ""
# ADD BASE CPP /nologo /MDd /W3 /Gm /GR /GX /ZI /Od /I "../../include" /D "_CRT_SECURE_NO_DEPRECATE" /D "WIN32" /YX /FD /GZ /c
# ADD CPP /nologo /MDd /W3 /Gm /GR /GX /ZI /Od /I "../../include" /D "_CRT_SECURE_NO_DEPRECATE" /D "WIN32" /YX /FD /GZ /c
# ADD BASE RSC /l 0x409 /d "_DEBUG"
# ADD RSC /l 0x409 /d "_DEBUG"
BSC32=bscmake.exe
# ADD BASE BSC32 /nologo
# ADD BSC32 /nologo
LINK32=link.exe
# ADD BASE LINK32 user32.lib gdi32.lib opengl32.lib glu32.lib /nologo /entry:"mainCRTStartup" /subsystem:console /incremental:yes /debug /machine:I386 /out:"../../lib/DebugLib/test_cyl.exe" /pdbtype:sept /libpath:"../../lib/DebugLib"
# ADD LINK32 user32.lib gdi32.lib opengl32.lib glu32.lib /nologo /entry:"mainCRTStartup" /subsystem:console /incremental:yes /debug /machine:I386 /out:"../../lib/DebugLib/test_cyl.exe" /pdbtype:sept /libpath:"../../lib/DebugLib"

!ELSEIF  "$(CFG)" == "test_cyl - Win32 ReleaseDLL"

# PROP BASE Use_MFC 0
# PROP BASE Use_Debug_Libraries 1
# PROP BASE Output_Dir "../../lib/ReleaseDLL"
# PROP BASE Intermediate_Dir "obj/cyl/ReleaseDLL"
# PROP BASE Target_Dir ""
# PROP Use_MFC 0
# PROP Use_Debug_Libraries 1
# PROP Output_Dir "../../lib/ReleaseDLL"
# PROP Intermediate_Dir "obj/cyl/ReleaseDLL"
# PROP Target_Dir ""
# ADD BASE CPP /nologo /MDd /W3 /Gm /GR /GX /ZI /Od /I "../../include" /D "_CRT_SECURE_NO_DEPRECATE" /D "WIN32" /YX /FD /GZ /c
# ADD CPP /nologo /MDd /W3 /Gm /GR /GX /ZI /Od /I "../../include" /D "_CRT_SECURE_NO_DEPRECATE" /D "WIN32" /YX /FD /GZ /c
# ADD BASE RSC /l 0x409 /d "_DEBUG"
# ADD RSC /l 0x409 /d "_DEBUG"
BSC32=bscmake.exe
# ADD BASE BSC32 /nologo
# ADD BSC32 /nologo
LINK32=link.exe
# ADD BASE LINK32 user32.lib gdi32.lib opengl32.lib glu32.lib /nologo /entry:"mainCRTStartup" /subsystem:console /incremental:yes /debug /machine:I386 /out:"../../lib/ReleaseDLL/test_cyl.exe" /pdbtype:sept /libpath:"../../lib/ReleaseDLL"
# ADD LINK32 user32.lib gdi32.lib opengl32.lib glu32.lib /nologo /entry:"mainCRTStartup" /subsystem:console /incremental:yes /debug /machine:I386 /out:"../../lib/ReleaseDLL/test_cyl.exe" /pdbtype:sept /libpath:"../../lib/ReleaseDLL"

!ELSEIF  "$(CFG)" == "test_cyl - Win32 DebugDLL"

# PROP BASE Use_MFC 0
# PROP BASE Use_Debug_Libraries 1
# PROP BASE Output_Dir "../../lib/DebugDLL"
# PROP BASE Intermediate_Dir "obj/cyl/DebugDLL"
# PROP BASE Target_Dir ""
# PROP Use_MFC 0
# PROP Use_Debug_Libraries 1
# PROP Output_Dir "../../lib/DebugDLL"
# PROP Intermediate_Dir "obj/cyl/DebugDLL"
# PROP Target_Dir ""
# ADD BASE CPP /nologo /MDd /W3 /Gm /GR /GX /ZI /Od /I "../../include" /D "_CRT_SECURE_NO_DEPRECATE" /D "WIN32" /YX /FD /GZ /c
# ADD CPP /nologo /MDd /W3 /Gm /GR /GX /ZI /Od /I "../../include" /D "_CRT_SECURE_NO_DEPRECATE" /D "WIN32" /YX /FD /GZ /c
# ADD BASE RSC /l 0x409 /d "_DEBUG"
# ADD RSC /l 0x409 /d "_DEBUG"
BSC32=bscmake.exe
# ADD BASE BSC32 /nologo
# ADD BSC32 /nologo
LINK32=link.exe
# ADD BASE LINK32 user32.lib gdi32.lib opengl32.lib glu32.lib /nologo /entry:"mainCRTStartup" /subsystem:console /incremental:yes /debug /machine:I386 /out:"../../lib/DebugDLL/test_cyl.exe" /pdbtype:sept /libpath:"../../lib/DebugDLL"
# ADD LINK32 user32.lib gdi32.lib opengl32.lib glu32.lib /nologo /entry:"mainCRTStartup" /subsystem:console /incremental:yes /debug /machine:I386 /out:"../../lib/DebugDLL/test_cyl.exe" /pdbtype:sept /libpath:"../../lib/DebugDLL"

!ENDIF

# Begin Target

# Name "test_cyl - Win32 ReleaseLib"
# Name "test_cyl - Win32 DebugLib"
# Name "test_cyl - Win32 ReleaseDLL"
# Name "test_cyl - Win32 DebugDLL"
# Begin Group "ode"

# PROP Default_Filter ""
# Begin Group "test"

# PROP Default_Filter ""
# Begin Source File

SOURCE=../../ode/test/test_cyl.cpp
# End Source File
# End Group
# End Group
# Begin Group "drawstuff"

# PROP Default_Filter ""
# Begin Group "src"

# PROP Default_Filter ""
# Begin Source File

SOURCE=../../drawstuff/src/resources.rc
# End Source File
# End Group
# End Group
# End Target
# End Project