echo ===> Building Galp core...
javac -d ./build/ ./src/java/*.java
if [ $? -ne 0 ]; then exit; fi

echo ====> Building Triggers package...
javac -d ./build/ -cp ./build/ ./src/java/trigger/*.java
if [ $? -ne 0 ]; then exit; fi

echo Finished.
echo
