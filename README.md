# eigen-lgsm-vs-kdl
The idea behind this repo is to aid the transition of `ocra-recipes` from `eigen-lgsm` to `KDL`. It does so, by performing running a set of tests (using Google Test) that compare the equivalency among the different operations used across the `ocra-recipes` library.

# Compilation
With the goal of having fully contained tests, this project will download also the necessary libraries such as `eigen`, `eigen-lgsm`, `googletest`, `orocos-kinematics-dynamics`. The following steps must be followed:

- Download this repo as:
`git clone https://github.com/jeljaik/eigen-lgsm-vs-kdl --recursive`. Notice the `recursive` option. This will download the aforementioned dependencies.
- Run the `configure.sh` script. First make it executable with `chmod 700 configure.sh`.
- Run the tests from the `build` directory automatically created in the previous step, i.e. `kdlVsLgsm`.
