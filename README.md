# ik-performance-test

## Required
+ [CMake](https://cmake.org)
+ [Eigen3](http://eigen.tuxfamily.org/index.php?title=Main_Page)
+ [qpOASES](https://github.com/coin-or/qpOASES)
+ [OSQP](https://github.com/oxfordcontrol/osqp)
+ [osqp-eigen](https://github.com/robotology/osqp-eigen)
+ [RBDL](https://github.com/rbdl/rbdl)

## Build
```Bash
$ mkdir build/
$ cd build/
$ cmake ..
$ make
```

## Run
```Bash
$ cd build/
$ ./bin/multiple_ik_benchmark
```

## Docker
```Bash
$ cd docker/
$ docker-compose up
```
