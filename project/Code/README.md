Ce programme graphique permet de lire et d'afficher les
fichier d'état `.stat` du projet PIN.

Vous devriez pouvoir le charger, le compiler, le modifier avec **QT Creator** 
sans modification. 

Pour le charger et le compiler avec **CLion**, il est possible
que vous deviez indiquer le chemin vers QT à CMake. Soit en
ajoutant une ligne du type 

`set(CMAKE_PREFIX_PATH "/Users/oce/Qt/6.5.2/macos/lib/cmake")`

au fichier `CMakeLists.txt`, soit en ajoutant une option du type 

`-DCMAKE_PREFIX_PATH=/Users/oce/Qt/6.5.2/macos/lib/cmake/`

dans les préférences de CLion, dans la section "Build, Execution, Deployement / CMake / CMake options"

