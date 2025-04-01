# Running the program

- There are scripts to allow you to run the following programs:
  - [`init.sh`](./init.sh) For downloading the repo the first time
  - [`xbox.sh`](./xbox.sh) runs the controller
  - [`runcar.sh`](./runcar.sh) runs the ORIN NX on the car
  - [`transfer.sh`](./transfer.sh) For transfering files over to the ORIN NX.

## Running the ros2 `laptop`/`controller` code

#### AS ALWAYS DON'T FORGET TO SOURCE ROS2 FIRST

### Prerequisite:

These prerequisite steps aren't necessary if you've built and installed these packages already. You will need to have the laptop package built and installed. If they aren't already installed use this command:

`colcon build --packages-select laptop`

### Steps:

1. Source the local setup:

`source install/setup.bash`

2. Run the code (controller):

`ros2 run laptop input_node controller`

(laptop):

`ros2 run laptop keyboard`

## Running the ros2 `mdc_car` code

#### AS ALWAYS DON'T FORGET TO SOURCE ROS2 FIRST

### Prerequisite:

These prerequisite steps aren't necessary if you've built and installed these packages already. You will need to have the mdc_car package built and installed. If they aren't already installed use this command:

`colcon build --packages-select mdc_car`

### Steps:

1. Source the local setup:

`source install/setup.bash`

2. Run the code

`ros2 run mdc_car main_control`
