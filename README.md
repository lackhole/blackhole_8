# Blackhole v.8

| 3D Ray Tracing                   | Blackhole with ordinary objects |
|:---------------------------------|---------------------------------|
| ![3d_simul](output/3d_simul.gif) | ![aespa](output/aespa.gif)      |

| Blackhole with a thin accretion disc | With disc and a background                                |
|:-------------------------------------|:----------------------------------------------------------|
| ![travel](output/travel.gif)         | ![disc_and_background](output/disc_and_background.gif)    |
#### Full Video at [YouTube](https://www.youtube.com/channel/UCc4TIUDXIz1nJD0Xs1_iwUQ)


## Requirements
* C++ 17
* CMake
* OpenCV
* ~~OpenCL~~

## Build
```
cmake -B build
cmake --build build
```

## Run
#### Ray tracing simulation (without a blackhole)
```
./build/ray_tracer_test
```
#### Single blackhole simulation
```
./build/blackhole_solution_test
```

### Camera Control(Experimental)
W: move forward  
S: move backward  
A: move left  
D: move right  
Z: move up  
X: move down  
Q: flip left  
E: flip right  
I: look down  
K: look up  
J: look left  
L: look right  
Press with shift: Move/Flip/Look faster  

-: Decrease FoV  
+: Increase FoV  

#### * Press ESC to quit

## Old version
* [Fixed camera-blackhole simulator](https://github.com/lackhole/blackhole_6)
  * Faster, but limited positions

## OpenCL support
In progress

## Multiple blackholes
In progress
