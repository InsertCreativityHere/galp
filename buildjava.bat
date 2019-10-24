@ECHO OFF
REM This prevents the commands in this file from being displayed on the command line.

ECHO     ====^> Generating directories...
REM Creates two new folders 'build\classes' and 'build\res' only if they don't already exist.
IF NOT EXIST ".\build\classes\" MKDIR ".\build\classes" && IF NOT EXIST ".\build\res\" MKDIR ".\build\res"
REM Checks if the last command worked, and stops the script if it didn't.
IF ERRORLEVEL 1 EXIT /B 1

ECHO     ====^> Compiling Galp core...
REM Compiles all the java code in the 'src\java' folder (NOT any subfolders), and puts them in the 'build\classes' folder.
javac -verbose -d "./build/classes/" "./src/java/"*.java
REM Checks if the last command worked, and stops the script if it didn't.
IF ERRORLEVEL 1 EXIT /B 1

ECHO     ====^> Compiling Triggers package...
REM Compiles all the java code in the 'src\java\trigger' folder, and puts them in the 'build\classes' folder.
javac -verbose -d "./build/classes/" -cp "./build/classes/" "./src/java/trigger/"*.java
REM Checks if the last command worked, and stops the script if it didn't.
IF ERRORLEVEL 1 EXIT /B 1

echo     ====^> Copying Manifest...
REM Makes a copy of the 'src\java-manifest.mf' file at 'build\res\manifest.mf'.
COPY /y /v ".\src\java-manifest.mf" "build\res\manifest.mf"
REM Checks if the last command worked, and stops the script if it didn't.
IF ERRORLEVEL 1 EXIT /B 1

echo     ====^> Building jar file...
REM Combines all the compiled java code and resources into a JAR executable file.
jar cfmv "./build/galp.jar" "./build/res/manifest.mf" -C "./build/classes" "."
REM Checks if the last command worked, and stops the script if it didn't.
IF ERRORLEVEL 1 EXIT /B 1

ECHO Finished!
ECHO.
