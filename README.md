# experimental_robotics_lab1
<!-- PROJECT SHIELDS -->
<!--
*** I'm using markdown "reference style" links for readability.
*** Reference links are enclosed in brackets [ ] instead of parentheses ( ).
*** See the bottom of this document for the declaration of the reference variables
*** for contributors-url, forks-url, etc. This is an optional, concise syntax you may use.
*** https://www.markdownguide.org/basic-syntax/#reference-style-links
-->
[![Contributors][contributors-shield]][contributors-url]
[![Forks][forks-shield]][forks-url]
[![Stargazers][stars-shield]][stars-url]
[![Issues][issues-shield]][issues-url]
[![MIT License][license-shield]][license-url]
[![LinkedIn][linkedin-shield]][linkedin-url]



<!-- PROJECT LOGO -->
<br />
<p align="center">
  <a href="https://github.com/FilipHesse/experimental_robotics_lab1">
    <img src="images/logo.png" alt="Logo" width="80" height="80">
  </a>

  <h3 align="center">ROBOT_PET</h3>

  <p align="center">
    This repository contains the first assignment of the experimental robotics laboratory course at the University of Genoa.
    It can be used to play around with ros. It contains only simulations, so no special hardware is needed.
    The doxygen-documantation of the code can be found here:
    <br />
    <a href="https://github.com/FilipHesse/experimental_robotics_lab1/docs/html"><strong> « « API » » </strong></a>
    <br />
    <br />
    <a href="https://github.com/FilipHesse/experimental_robotics_lab1/issues">Report Bug</a>
    ·
    <a href="https://github.com/FilipHesse/experimental_robotics_lab1/issues">Request Feature</a>
  </p>
</p>



<!-- TABLE OF CONTENTS -->
## Table of Contents

- [experimental_robotics_lab1](#experimental_robotics_lab1)
  - [Table of Contents](#table-of-contents)
  - [About The Project](#about-the-project)
    - [Built With](#built-with)
  - [Software architecture](#software-architecture)
    - [Component Diagram](#component-diagram)
    - [Launchfiles](#launchfiles)
    - [Ros parameters](#ros-parameters)
    - [Messages, services and actions](#messages-services-and-actions)
    - [State Machine](#state-machine)
  - [Getting Started](#getting-started)
    - [Prerequisites](#prerequisites)
    - [Installation](#installation)
  - [Usage](#usage)
  - [Roadmap](#roadmap)
  - [Contributing](#contributing)
  - [License](#license)
  - [Acknowledgements](#acknowledgements)



<!-- ABOUT THE PROJECT -->
## About The Project

The aim of this assignemt is to get acquainted with building finite state machines
with using smach and designing an appropritate software architecture for the following scenario:
A robot pet (which is simulated in our case) has 3 states: 
* normal: The robot moves around randomly
* play: The robot approaches the person, waits for a position command, goes to that position and comes back to the user
* sleep: The robot returns to the position of a house  and sleeps for some time, then it wakes up and returns to normal

### Built With

* [ROS noetic](http://wiki.ros.org/noetic/Installation)
* [Python3](https://www.python.org/downloads/)
* [Smach](http://wiki.ros.org/smach)

## Software architecture

### Component Diagram
![Component Diagram](./docs/diagrams/EXP_ASS1_UML_Component_diagram.jpg)

The ros package robot_pet consits of 6 components which are 
* <strong>ui</strong> : 
  * This node is the simulated user interface. It is a service client, that creates commands to simulate the users behavior. The programmer has chosen a service over a publisher, because we want to
  make sure no message gets lost.
  It creates and sends two types of commands:
    1) "play" 0 0 to notify the robot to go to playing mode
    2) "go_to" x y to give the robot a target position.
  * Each fifth command is a play command, the other commands are go_to commands.
  Between two commands, there is always a rondom time passing between 0.5 and 5 seconds.
  * The commands are sent with a random time delay between 0.5 and 5 seconds
  * Requires ROS parameters: 
    * /map_width
    * /map_height
* <strong>behavior_state_machine</strong> :
  * This is the heart of robot_pet package, which defines the robots behavior
  * Contains a finite state machine implemented in smach. The 3 states of the
  robot pet are NORMAL, PLAY and SLEEP. The state diagram can be found below
  * Each interface with the ROS infrastructure, such as service clients,
  servers, action clients and publishers are implemented within separate
  classes. All these interfaces are then passed to the smach-states while they
  are constructed, in order to make the interfaces accessible for the states.
  * Requires ROS parameters: 
    * /map_width
    * /map_height
* <strong>localizer_navigator</strong> :
  * Simple ROS-node, which simulates the robot naviagation. It simulates to localize the robot and navigates to next target point
  *  It contains an action server, that can receive a new target position. The
  server simply waits for some time and considers the position to be reached.
  Then the positions is published to the topic pet_position.
  * The programmer has chosen an action server instead of a service, because the
  action server does not block the client. This way the client can continue
  working while the result of this action is computed.
* <strong>user_localizer</strong> :
  * This node simulates a user moving in the environment. It publishes the new user position at a low frequency of 0.25 Hz. Each time the position is changed, the user has not moved further than one step in x and one step in y.
  * Requires ROS parameters: 
    * /map_width
    * /map_height
    * /user_pos_x
    * /user_pos_y
* <strong>map</strong> :
  * The map node contains all the knowledge about the map itself, which is simply a rectangular grid with integer positions. It contains information about the dimensions of the map and the positions of all objects: pet, user, house, pointer. 
  * The three subscribers subscribe to the variable positions of the objects in
  the map: user, pet, pointer. The server provides the service get_position to the ros environment. The publisher publishes an image of the current map each time the map is updated.
  * Requires ROS parameters: 
    * /map_width
    * /map_height
    * /user_pos_x
    * /user_pos_y
    * /user_house_x
    * /user_house_y
    * /user_pet_x
    * /user_pet_y
* <strong>rqt_image_view</strong> :
  * That node is a built in ROS node, so it has not been implemented in this context. It is used to display the current positions of the actors: House, Pet, User, Pointer
  
  ![Visualization](./docs/screenshots/visualization.png)

### Launchfiles

Tho launchfiles are present in the robot_pet package:
* run_system.launch: This file launches all the above nodes with some default 
ROS parameters.
* params.launch: This launchfile only sets default ROS parameters. This enables the user run single nodes, that may require sompe parameters
  
### Ros parameters

Several ROS parameters can be set to modify this software. 
* /map_width: Dimension of rectangular map in x-direction
* /map_height: Dimension of rectangular map in y-direction
* /user_pos_x: Startposition of user: x
* /user_pos_y: Startposition of user: y
* /user_house_x: Startposition of house: x
* /user_house_y: Startposition of house: y
* /user_pet_x: Startposition of pet: x
* /user_pet_y: Startposition of pet: y
  
### Messages, services and actions

The following <strong>ROS-messages</strong> have been defined:
* Point2d.msg: Contains x and y coordinates of a point
```sh
int64 x
int64 y
```
* Point2d.msg: Contains a Point2d and a flag on. The additional flag specifies, if the pointer position should be displayed or not
```sh
robot_pet/Point2d point
bool on
```
The following <strong>ROS-services</strong> have been defined:
* GetPosition.srv: The caller specifies a string ("pet", "user", "house", "pointer") of the object. If the specified object is part of the map, the return value is success = True and the point contains the according coordinates. Else the success flag is False.
```sh
Header header
string object 
---
bool success
robot_pet/Point2d point
```

* PetCommand.srv: A string command ("play", "go_to") can be sent. If the command is "go_to", then a targetpoint should be specified. If the command is "play", the point is ignored.
```sh
Header header
string command 
robot_pet/Point2d point
---
```
The following <strong>ROS-action</strong> has been defined:

* SetTargetPositionAction: This action is used to set a target position of the robot. The caller pecifies a target, which is a Point2d (see above). When the position is reached, the action server confirms the final position by sending back the target point.
```sh
#Goal
robot_pet/Point2d target
---
#Result
robot_pet/Point2d final_position
---
#Feedback
```

### State Machine
![State Diagram](./docs/diagrams/EXP_ASS1_UML_State_diagram.jpg)
The above state diagram shows clearly the three states of the system:
* NORMAL: The robot moves randomly from one position to another. It can transition to the PLAY state by receiving a user command to play. If the sleeping timer triggers sleeping time, then the state transitions to SLEEP.
* SLEEP: The robot approaches the house and stays there, until the sleeping timer triggers, that it's time to wake up (transition "slept_enough"). Then the robot returns to the state NORMAL
* PLAY: The robot performs the following actions in a loop:
  1) Go to user
  2) Wait for a command that specifies a new target
  3) Go to new target
  4) Repeat
  * When a random number of games has been reached, the robot stops playing ("played_enough") and returns to the normal state. When the sleeping timer triggers time to sleep, the state transitions to SLEEP.


<!-- GETTING STARTED -->
## Getting Started

To get a local copy up and running follow these simple steps.

### Prerequisites

This is an example of how to list things you need to use the software and how to install them.
* npm
```sh
npm install npm@latest -g
```

### Installation

1. Clone the repo
```sh
git clone https://github.com/FilipHesse/experimental_robotics_lab1.git
```
2. Install NPM packages
```sh
npm install
```



<!-- USAGE EXAMPLES -->
## Usage

Use this space to show useful examples of how a project can be used. Additional screenshots, code examples and demos work well in this space. You may also link to more resources.

_For more examples, please refer to the [Documentation](https://example.com)_



<!-- ROADMAP -->
## Roadmap

See the [open issues](https://github.com/FilipHesse/experimental_robotics_lab1/issues) for a list of proposed features (and known issues).



<!-- CONTRIBUTING -->
## Contributing

Contributions are what make the open source community such an amazing place to be learn, inspire, and create. Any contributions you make are **greatly appreciated**.

1. Fork the Project
2. Create your Feature Branch (`git checkout -b feature/AmazingFeature`)
3. Commit your Changes (`git commit -m 'Add some AmazingFeature'`)
4. Push to the Branch (`git push origin feature/AmazingFeature`)
5. Open a Pull Request



<!-- LICENSE -->
## License

Distributed under the MIT License. See `LICENSE` for more information.



Project Link: [https://github.com/FilipHesse/experimental_robotics_lab1](https://github.com/FilipHesse/experimental_robotics_lab1)



<!-- ACKNOWLEDGEMENTS -->
## Acknowledgements

* []()
* []()
* []()





<!-- MARKDOWN LINKS & IMAGES -->
<!-- https://www.markdownguide.org/basic-syntax/#reference-style-links -->
[contributors-shield]: https://img.shields.io/github/contributors/FilipHesse/experimental_robotics_lab1.svg?style=flat-square
[contributors-url]: https://github.com/FilipHesse/experimental_robotics_lab1/graphs/contributors
[forks-shield]: https://img.shields.io/github/forks/FilipHesse/experimental_robotics_lab1.svg?style=flat-square
[forks-url]: https://github.com/FilipHesse/experimental_robotics_lab1/network/members
[stars-shield]: https://img.shields.io/github/stars/FilipHesse/experimental_robotics_lab1.svg?style=flat-square
[stars-url]: https://github.com/FilipHesse/experimental_robotics_lab1/stargazers
[issues-shield]: https://img.shields.io/github/issues/FilipHesse/experimental_robotics_lab1.svg?style=flat-square
[issues-url]: https://github.com/FilipHesse/experimental_robotics_lab1/issues
[license-shield]: https://img.shields.io/github/license/FilipHesse/experimental_robotics_lab1.svg?style=flat-square
[license-url]: https://github.com/FilipHesse/experimental_robotics_lab1/blob/master/LICENSE.txt
[linkedin-shield]: https://img.shields.io/badge/-LinkedIn-black.svg?style=flat-square&logo=linkedin&colorB=555
[linkedin-url]: https://linkedin.com/in/FilipHesse
[product-screenshot]: images/screenshot.png