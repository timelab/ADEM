There is a <LibraryName> folder per library, containing:
* (optional) keywords.txt for code highlighting
* <LibraryName>.cpp
* <LibraryName>.h

For tips coding the Library, see http://playground.arduino.cc/Code/Library

To use the ADEM libraries, you can eiter:
* On *nix, the Arduino Libraries reside in ~/Arduino/libraries/. If you create a symbolic link, it will keep your libaries up to date with your local github folder.
```
        ln -s ~/<your ADEM github folder>/arduino/libraries/Sensor ~/Arduino/libraries
        ln -s ~/<your ADEM github folder>/arduino/libraries/Ppd42 ~/Arduino/libraries
```
* Import the libraries in Arduino IDE. This will make a point-in-time copy.
  * Schets > Bibliotheek gebruiken > .ZIP Bibliotheek toevoegen...
  * Select the folder of the library (~/<your ADEM github folder>/arduino/libraries/Sensor) > OK
