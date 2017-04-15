#clear Terminal
reset

#cleaning previous java classes
echo "Cleaning Previous java classes:"
echo "-------------------------------"
rm -rf wrapper/*
# -r ->recursive, to include directories and sub-directories
# -f -> force , ignore non-existant files and never prompt user  

#run swig to generate C++ wrapper and java classes
echo "Running swig to generate java files and cxx files:"
echo "--------------------------------------------------"
swig -c++ -java -outdir wrapper -package wrapper -o cpp/wrap_pcl.cxx -I/usr/include/pcl-1.7 -I/usr/include/eigen3 -importall  PCL.i
# -c++ -> c++ mode (if we omitted it will operate in c mode)
# -java -> output language wrapper
# -outdir java/pcl  -> output Directory is java/pcl
# -package pcl -> top level package name is pcl
# -o cpp/wrap_pcl.cxx -> output C++ wrapper code file path is "cpp/wrap_pcl.cxx"
# -I/usr/include/pcl-1.7 -> include /usr/include/pcl-1.7 in the directories search pathes (pcl library header path)
# -v -> verbose mode (display every step)
# -importall -> look at all #include for type information and use it if you need
# PCL.i -> interface file that is used with swig

#execting the correct script to correct swig bug
echo ""
echo "executing correction script (correcting some bugs from SWIG):"
echo "-------------------------------------------------------------"
sudo python3 correct_script.py wrapper wrapper/

#compiling java classes
echo ""
echo "Compiling java classes:"
echo "-----------------------"
cd wrapper
javac *.java pcl/*.java pcl/search/*.java
# -verbose -> display all information
cd .. 

#compile C++ wrapper library
echo ""
echo "Genertaing the solution and compiling it using cmake and make:"
echo "--------------------------------------------------------------"
cd cpp
#building the project with CMake with default build target
#try to make it build with Unix files make
cmake --build CMake 
cd ..

#Creating library jar
echo ""
echo "Generating jar file:"
echo "--------------------"
jar cf PCL_Wrapper.jar wrapper/*.class wrapper/pcl/*.class wrapper/pcl/*/*.class
# cvf ->  v=verbose , cf = create file 
cd ..   
