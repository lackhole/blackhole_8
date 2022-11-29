project(root)
add_library(root INTERFACE)

find_package(ROOT REQUIRED)

target_link_libraries(root INTERFACE
    ROOT::Core
    ROOT::Imt
    ROOT::RIO
    ROOT::Net
    ROOT::Hist
    ROOT::Graf
    ROOT::Graf3d
    ROOT::Gpad
    ROOT::ROOTVecOps
    ROOT::Tree
    ROOT::TreePlayer
    ROOT::Rint
    ROOT::Postscript
    ROOT::Matrix
    ROOT::Physics
    ROOT::MathCore
    ROOT::Thread
    ROOT::MultiProc
    ROOT::ROOTDataFrame)
