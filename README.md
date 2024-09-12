# Template Robot
> A template to build future robots off of.

## Contains
- Gazebo simulator setup
- Working devcontainer
- Basic swerve drive and command based RIO
- Useful scripts
- ROS setup and tools

## Changing for your robot
1. Rename `REPO_NAME` across files to the name of your repository.


# Contributing

## Process

Code needs to be satisfy the following two points:
1. Be *general*. It shouldn't be anything specific for a particular year, such as specific parts or pieces of a robot.
2. Be *tested*. The code needs to have been already run on a specific robot, ideally in a competition.

## Style Guide

For python files (mainly the `rio` folder), make a best effort to follow [Python PEP 8](https://peps.python.org/pep-0008/), which means that:
1. Files should have all lowercase names with underscores if needed
2. Folders should have all lowercase names without underscores
3. Class names should use PascalCase