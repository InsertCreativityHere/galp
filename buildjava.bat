@ECHO OFF

ECHO ====> Generating directories...
MKDIR ./build/classes
MKDIR ./build/res
IF ERRORLEVEL 1 EXIT

ECHO ====> Compiling Galp core...
javac -d ./build/classes/ ./src/java/*.java
IF ERRORLEVEL 1 EXIT

ECHO ====> Compiling Triggers package...
javac -d ./build/classes/ -cp ./build/classes/ ./src/java/trigger/*.java
IF ERRORLEVEL 1 EXIT

echo ====> Copying Manifest...
COPY ./src/java/manifest.mf ./build/res/manifest.mf
IF ERRORLEVEL 1 EXIT

echo ====> Building jar file...
jar cfm ./build/galp.jar ./build/res/manifest.mf -C ./build/classes .
IF ERRORLEVEL 1 EXIT

ECHO Finished!
ECHO
