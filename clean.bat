@ECHO OFF
REM This prevents the commands in this file from being displayed on the command line.

REM Moves into the 'build' folder
CD build
REM Deletes every file inside the 'builds' folder except those ending in '.gitkeep'
FOR /R %%f IN ("*") DO IF NOT %%~xf==.gitkeep (
    ECHO Deleting file %%f && DEL /Q %%f
)
REM Deletes every folder inside the 'builds' folder.
FOR /D %%d IN ("*") DO ECHO Deleting folder %%d && RMDIR /S /Q %%d
REM Moves out of the 'build' folder back into 'galp'
CD ..

ECHO Finished.
ECHO.
