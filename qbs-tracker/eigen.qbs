import qbs

Module {
    Depends { name: "cpp" }
    property string eigenPath: "c:/src/eigen"
    cpp.includePaths: eigenPath
    //cpp.libraryPath: xyzPath + "/lib"
    //cpp.staticLibraries: "xyz"
}
