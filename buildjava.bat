@ECHO OFF

ECHO Building Galp core...
javac -d ./build/ ./src/java/*.java

ECHO Building Triggers package...
javac -d ./build/ ./src/java/trigger/*.java

ECHO Finished.
ECHO
