echo "    ====> Generating directories..."
# Creates two new folders 'build/classes' and 'build/res' only if they don't already exist.
mkdir -pv "./build/classes" && mkdir -pv "./build/res"
# Checks if the last command worked, and stops the script if it didn't.
if [ $? -ne 0 ]; then exit; fi

echo "    ====> Compiling Galp core..."
# Compiles all the java code in the 'src/java' folder (NOT any subfolders), and puts them in the 'build/classes' folder.
javac -verbose -d "./build/classes/" "./src/java/"*.java
# Checks if the last command worked, and stops the script if it didn't.
if [ $? -ne 0 ]; then exit; fi

echo "    ====> Compiling Triggers package..."
# Compiles all the java code in the 'src/java/trigger' folder, and puts them in the 'build/classes' folder.
javac -verbose -d "./build/classes/" -cp "./build/classes" "./src/java/trigger/"*.java
# Checks if the last command worked, and stops the script if it didn't.
if [ $? -ne 0 ]; then exit; fi

echo "    ====> Copying Manifest..."
# Makes a copy of the 'src/java-manifest.mf' file at 'build/res/manifest.mf'.
cp -v "./src/java-manifest.mf" "./build/res/manifest.mf"
# Checks if the last command worked, and stops the script if it didn't.
if [ $? -ne 0 ]; then exit; fi

echo "    ====> Building Galp jar file..."
# Combines all the compiled java code and resources into a JAR executable file.
jar cfmv "./build/galp.jar" "./build/res/manifest.mf" -C "./build/classes" "."
# Checks if the last command worked, and stops the script if it didn't.
if [ $? -ne 0 ]; then exit; fi

echo "Finished!"
echo
