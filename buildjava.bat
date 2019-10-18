@ECHO OFF

ECHO ====> Building Galp core...
javac -d ./build/ ./src/java/*.java
IF ERRORLEVEL 1 EXIT

ECHO ====> Building Triggers package...
javac -d ./build/ -cp ./build/ ./src/java/trigger/*.java
IF ERRORLEVEL 1 EXIT

ECHO Finished.
ECHO
