# Running the program

## ROS Setup

Make sure the `ROS_DOMAIN_ID` is the same for all modules (NX, Laptop, NANO).

```sh
echo $ROS_DOMAIN_ID # Example: 42
export ROS_DOMAIN_ID = 42 # Only those with the same domain will ros2 topic list show the topics
```

To access `ros2 run ...` and `ros2 topic list`, etc you must first source ros

```sh
source /opt/ros/humble/setup.sh
```

## Scripts

There are scripts to allow you to run the following programs. Look at each individual script to see what is done:

- [`init.sh`](./init.sh) For downloading the repo the first time
  - [`initAzure.sh`](./initAzure.sh) For initializing the Azure Kinect DK.
- [`xbox.sh`](./xbox.sh) runs the controller. Requires `pip install pygame`
- [`runcar.sh`](./runcar.sh) runs the ORIN NX on the car
- [`transfer.sh`](./transfer.sh) For transfering files over to the ORIN NX.
