echo ====\> Generating directories...
mkdir ./build/classes
mkdir ./build/res
if [ $? -ne 0 ]; then exit; fi

echo ====\> Compiling Galp core...
javac -d ./build/classes/ ./src/java/*.java
if [ $? -ne 0 ]; then exit; fi

echo ====\> Compiling Triggers package...
javac -d ./build/classes/ -cp ./build/classes ./src/java/trigger/*.java
if [ $? -ne 0 ]; then exit; fi

echo ====\> Copying Manifest...
cp ./src/java-manifest.mf ./build/res/manifest.mf
if [ $? -ne 0 ]; then exit; fi

echo ====\> Building jar file...
jar cfmv ./build/galp.jar ./build/res/manifest.mf -C ./build/classes .
if [ $? -ne 0 ]; then exit; fi

echo Finished!
echo
