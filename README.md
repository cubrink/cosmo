![cosmo logo](/assets/cosmo_logo.png)

#

Devpost Submission: https://devpost.com/software/cosmo-qw79rc

Project Video: https://www.youtube.com/watch?v=VRjiT6nPFoY

## Inspiration

We heard that the theme this year was space and we wanted to go all out. This project was unique because we recruited a team for the hackathon that had a wide range of skills, from mechanical engineering all the way to computer science. After some brainstorming, we decided we wanted to build something that would require all of our skillsets and decided on an interactive space exploration simulator.

## What it does

Cosmo is a interactive space exploration simulator. The custom built flight controller takes account of your orientation as you rotate freely along one axis. This gives an immersive and responsive experience as you explore the cosmos.

## How we built it

This project had a lot of moving parts. Here is a run down of the different parts that went into making this possible:

### The Cockpit

This is where the magic happens. There are three key systems here, the inner cockpit, the outer frame, and the motor pulley assembly. 

We started by using an outer frame that we already had at our disposal. For the inner cockpit was entirely fabricated with steel, scrap wood and an old office chair. We located the center of mass and placed pivot shafts so the cockpit could rotate freely while someone was strapped in. Finally, for the motor assembly we used two spare electric motors with 1/3 hp and 1725 rpms, this was geared down using a pulley system, bringing it to roughly 1.3 rps. 

From here, we just needed to integrate the electronics to control the system from the flight controller.

### Flight Controller

This is the brains of the operation. It is responsible for relaying information from the pilot to the other systems that make cosmo possible. The first task of the flight controller is to take inputs from the user and communicate with the simulator by acting as a Bluetooth gamepad. The flight controller uses adaptive controls that translate joystick movements relative to the orientation of the cockpit.

This component also is serving a second purpose as the pilot rolls their spacecraft. The flight controller houses the logic for the activation of the motors and transmits that information to the motor controller using a lightweight wireless messaging protocol called ESP-Now. A motor is activated only when the other motor has stopped and the cockpit's angular velocity has subsided. This functionality is critical as it prevents the motors from burning out.

### Motor Controller

This is what drives the action. The motor controller has one main purpose, switch between motors to control the rotation of the cockpit. Doing this right requires several different pieces. The first of which are the relays that act as a electronically controlled mechanical switch. This allows us to programmatically control which motor is running. The second piece in the puzzle are the speed controllers. These change the input frequency of the motors, allowing for a slower turning rate.

This is all controlled by a ESP32 microcontroller which communicates wirelessly with the flight controller and activates motors when it is signaled. To prevent runaway scenarios, the microcontroller frequently pings the flight controller and disables all motors if the message is not received.

### Space Engine

This was our simulation of choice as it natively supports Bluetooth gamepads and it provides vast and realistic universe to explore. It simulates real astronomical objects and fills in the gaps with scientifically accurate procedurally generated data. This provides an excellent environment to learn and be amazed! 

## Challenges we ran into

The first challenged we faced was that because the cockpit was freely rotating, it had to be self isolated as any external wires would tangle. To solve this we relied on wireless communication to let each part of our project interact. Our first attempt was for all devices to communicate with Bluetooth, however we had troubles with consistently establish a connection with the motor controller. We solved this by moving to the ESP-Now protocol, which was much lighter weight and consistent.

The second challenge we ran into was finding a way to translate joystick inputs based off of the orientation of the cockpit. We were tracking orientation using a accelerometer, but the first algorithms we found online didn't work properly with 360° rotations. After further research, we found a library for our accelerometer that was able to support arbitrary rotations.

Mechanically speaking, this project was challenging because we didn't have access to all the ideal tools needed. We did our best to source as many tools as possible that would help during this hackathon, but there's only so much you can do as a college student.

## Accomplishments that we're proud of

We're really proud of how large of a project we completed in a weekend! There were a lot of moving parts (literally!) but we worked efficiently to get the project done. We were also proud of how we distributed the work. Our team consisted of mechanical engineering, electrical engineering, computer engineering and computer science. There was a lot of breadth to our skillsets and that strengthened our team.

## What we learned

Due to diverse skillsets present on our team, everyone was able to expand their skillset, by both being the mentor and the mentee. There was a lot of concepts to learn from in this project, including mechanical design, controlling AC motors, working with ESP32 microcontrollers, and managing a distributed wireless system.

## What's next for cosmo

In the future we'd like to beef up some of the parts used. More time to improve the pulley system would be very beneficial.  Allowing for more gamepad buttons on the flight controller would be nice, most gamepads have more than 8 buttons, but we were becoming limited by the space on the board.

Something that we'd love to add would be an additional axis of rotation, however, a 2-axis system would not have been possible in this timeframe.

