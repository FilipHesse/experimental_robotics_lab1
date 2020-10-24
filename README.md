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
* <strong>behavior_state_machine</strong> :
  * This is the heart of robot_pet package, which defines the robots behavior
  * Contains a finite state machine implemented in smach. The 3 states of the
  robot pet are NORMAL, PLAY and SLEEP. The state diagram can be found below
  * Each interface with the ROS infrastructure, such as service clients,
  servers, action clients and publishers are implemented within separate
  classes. All these interfaces are then passed to the smach-states while they
  are constructed, in order to make the interfaces accessible for the states.
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
* <strong>map</strong> :
  * The map node contains all the knowledge about the map itself, which is simply a rectangular grid with integer positions. It contains information about the dimensions of the map and the positions of all objects: pet, user, house, pointer. 
  * The three subscribers subscribe to the variable positions of the objects in
  the map: user, pet, pointer. The server provides the service get_position to the ros environment. The publisher publishes an image of the current map each time the map is updated.
* <strong>rqt_image_view</strong> :
  * That node is a built in ROS node, so it has not been implemented in this context. It is used to display the current positions of the actors:
  
  <kbd>
  ![Screenshot](./docs/screenshots/visualization.png)
  </kbd>
### State Machine
![State Diagram](./docs/diagrams/EXP_ASS1_UML_State_diagram.jpg)



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