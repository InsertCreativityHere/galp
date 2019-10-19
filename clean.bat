REM TODO I NEED TO TEST THESE BUILD SCRIPTS ON WINDOWS AND ADD VERBOSE MODES WHERE NEEDED!

REM This prevents the commands in this file from being displayed on the command line.
@ECHO OFF

ECHO     ====> Cleaning build files...

REM Deletes every file and folder inside the 'builds' folder.
rmdir /s /q "./build/*"

ECHO Finished.
ECHO
