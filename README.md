## Jass setup

1. Being in the directory with the project build project on robot `dts devel build -f -H <BOT_NAME>`
2. Being in the directory with the project run project `dts devel run -M -f -H <BOT_NAME> -L jass`
3. Open virtual joystick `dts duckiebot keyboard_control <BOT_NAME`
4. Press `A`



# Usage

```
git checkout daffy
dts devel build  # optional: -f --no-cache
dts devel run
```

# Template: template-core

This template provides a boilerplate repository
for developing ROS-based software in Duckietown.
Unlike the `template-ros` repository, this template
builds on top of the module 
[`dt-core`](https://github.com/duckietown/dt-core).
This is needed when your application requires access 
to tools and libraries defined in 
[`dt-core`](https://github.com/duckietown/dt-core).


**NOTE:** If you want to develop software that does not use
ROS, check out [this template](https://github.com/duckietown/template-basic).


## How to use it

### 1. Fork this repository

Use the fork button in the top-right corner of the github page to fork this template repository.


### 2. Create a new repository

Create a new repository on github.com while
specifying the newly forked template repository as
a template for your new repository.


### 3. Define dependencies

List the dependencies in the files `dependencies-apt.txt` and
`dependencies-py3.txt` (apt packages and pip packages respectively).


### 4. Place your code

Place your code in the directory `/packages/` of
your new repository.


### 5. Setup launchers

The directory `/launchers` can contain as many launchers (launching scripts)
as you want. A default launcher called `default.sh` must always be present.

If you create an executable script (i.e., a file with a valid shebang statement)
a launcher will be created for it. For example, the script file 
`/launchers/my-launcher.sh` will be available inside the Docker image as the binary
`dt-launcher-my-launcher`.

When launching a new container, you can simply provide `dt-launcher-my-launcher` as
command.


