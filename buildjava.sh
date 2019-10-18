echo Building Galp core...
javac -d ./build/ ./src/java/*.java

echo Building Triggers package...
javac -d ./build/ ./src/java/trigger/*.java

echo Finished.
echo
