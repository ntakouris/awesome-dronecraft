# awesome-dronecraft

> I originally created this as a short to-do list of study topics for becoming a software engineer,
> I am creating this list to share my 3 year list of topics that I studied on the side during the curriculum of a Computer Engineering and Informatics degree.
> This is a list of short and medium length study topics to obtain knowledge regarding autonomous rotorcraft.
> The items listed here will give you enough knowledge to be able to understand how they work,
> their limitations and effort required to make them fly.
> *Happy studying!*

Format of this is hugely inspired by [jwasham/coding-interview-university](https://github.com/jwasham/coding-interview-university)

Contributions are welcome, so please open a PR if you can improve this list or something is missing!

## What is it?

This is my big study plan for going from simple programmer dude to software engineer who can understand how autonomous drones work.

![Skydio's SLAM](https://pbs.twimg.com/ext_tw_video_thumb/976148551779500032/pu/img/erxg20ZxAfOoDBKb.jpg)

This is meant for **software engineers** or people who already know how to code and also got basic knowledge of computer science topics including math __(probablility, calculus, linear algebra)__. If you have many years of experience this can be easier to read through, but this is not guaranteed.

Disclaimer: I am not employed nor have I got any work experience on commerial or research rotorcraft.

---

## Table of Contents

- [What is it?](#what-is-it)
- [Why use it?](#why-use-it)
- [How to use it](#how-to-use-it)
- [Don't feel you aren't smart enough](#dont-feel-you-arent-smart-enough)
- [You can't pick only one language](#you-cant-pick-only-one-language)
- [Book List](#book-list)
- [Before you Get Started](#before-you-get-started)
- [What you Won't See Covered](#what-you-wont-see-covered)
- [Prerequisite Knowledge](#prerequisite-knowledge)
- [The Daily Plan](#the-daily-plan)
- WIP

## Why use it?

When I started this project, I didn't know how a drone could see, how the math behind the control
algorithm worked, what the limitations are or anything like that.
I started skimming through any resources I could after being inspired by the newly released
[Skydio r1](https://www.skydio.com/) and it's CEO's [numerous guest lectures at youtube](https://www.youtube.com/watch?v=ZI66eq7Nn1E). I slowly started building enough sensors to be able to grasp every sub-component of such flying systems. I have not learned everything yet, but the list is big enough to share and will always be evolving with more stuff added.

It's a long plan. It may take you months. If you are familiar with a lot of this already it will take you a lot less time.

## How to use it

<details>
<summary>How to use it</summary>

Everything below is an outline, and you should tackle the items in order from top to bottom.

I'm using Github's special markdown flavor, including tasks lists to check progress.

**Create a new branch so you can check items like this, just put an x in the brackets: [x]**


    Fork a branch and follow the commands below

`git checkout -b progress`

`git remote add zarkopafilis https://github.com/zarkopafilis/awesome-dronecraft`

`git fetch --all`

    Mark all boxes with X after you completed your changes

`git add .`

`git commit -m "Marked x"`

`git rebase zarkopafilis/master`

`git push --force`

[More about Github-flavored markdown](https://guides.github.com/features/mastering-markdown/#GitHub-flavored-markdown)

</details>

## Don't feel you aren't smart enough

- Successful software engineers are smart, but many have an insecurity that they aren't smart enough.
- [The myth of the Genius Programmer](https://www.youtube.com/watch?v=0SARbwvhupQ)
- [It's Dangerous to Go Alone: Battling the Invisible Monsters in Tech](https://www.youtube.com/watch?v=1i8ylq4j_EY)
- [Believe you can change](http://www.aaronsw.com/weblog/dweck)
- [Think you're not smart enough to work at Google? Well, think again](https://www.youtube.com/watch?v=uPOJ1PR50ag)

## You can't pick only one language

You can't use only 1 language you are comfortable in to do the coding of the drones. You'd have to rely on many tools. Here are some for example.

- C++
- C
- Python
- Matlab (Yes, I know)

You may also need to learn a bit of other tools in the proccess. This is NOT something that you should feel worried about. With studying consistency there is nothing you can't achieve!

## Book List

- This is meant to be a table of contents for the internet 'book'.
- WIP

## Before you Get Started

This list grew over many months, and yes, it kind of got out of hand.

Here are some mistakes I made so you'll have a better experience.

### You can't take it all in at once

Take breaks, write down possible things you need to re-review and watch them later or in a different way. This is a lot to take in and breaks are mandatory.

Some concepts can take much longer to understand fully. So if you don't understand it immediately do not worry. I suggest re-studying some of the parts that seemed hard to you after doing one pass on this study list.

### Focus

There are a lot of distractions that can take up valuable time. Focus and concentration are hard.

## What you won't see covered

- A lot of electorlogical stuff
- WIP

## Prerequisite Knowledge

<details>
<summary>Prerequisite Knowledge</summary>

Learn to code. This is only required for you to understand how the different algorithms and techniques that will be presented are actually implemented. You need this knowledge to be able to understand the source code of the popular data structure implementations, open source flight controllers and more. Basic data structure and algorithmic complexity should be included.

- [ ] **C, C++, Python, Anything**
    - Information available on the internet is widely available and you can find lot's of stuff by googling.
    - I could recommend to start with Python and work your way down to C and C++.
    - You can learn to code in parallel with this study plan, but things will be a lot harder and will take more time.
    - [] [A complete computer science study plan to become a software engineer](https://github.com/jwasham/coding-interview-university)

The next part is needed to be able to understand the math behind the stuff we are going to use. It's not at all harder compared to other fields of study. Hang on and in the end, you'll be surprised by how easy it is to understand everything. This is not the only path to learn these, it's just what I would take. (Yes, I like university lectures and whitepapers.)

- [ ] **Basic linear algebra, calculus, probablility and elementary physics**
    - [ ] [Single Variable Calculus - MIT OCW](https://ocw.mit.edu/courses/mathematics/18-01sc-single-variable-calculus-fall-2010/)
    - [ ] [Multivariable Calculus - MIT OCW](https://ocw.mit.edu/courses/mathematics/18-02sc-multivariable-calculus-fall-2010/)
    - [ ] [Gilbert Strang's Linear Algebra - MIT OCW](https://ocw.mit.edu/courses/mathematics/18-06-linear-algebra-spring-2010/) __there is really no better way to learn this in the world__
    - [ ] [Introduction to Probability and Statistics-  MIT OCW](https://ocw.mit.edu/courses/mathematics/18-05-introduction-to-probability-and-statistics-spring-2014/)
    - [ ] [Digital Signal Proccessing - Rensselaer Polytechnic Institute](https://www.youtube.com/playlist?list=PLuh62Q4Sv7BUSzx5Jr8Wrxxn-U10qG1et)

</details>

## Simulation and Control

<details>
<summary>Simulation and Control</summary>

- This is the basis of rotorcraft that is going to get covered first. These few resources will make you understand what a drone needs to fly bad, good, with the help of extra autonomy engines or with the help of a pilot.
- After this part, the corks and screws of each subsystem is going to be investigated thouroughly.

- [ ] [Matlab Tech Talks - Understanding Control Systems](https://www.mathworks.com/videos/series/understanding-control-systems-123420.html)
- [ ] [Matlab Tech Talks - Drone Simulation and Control](https://www.mathworks.com/videos/drone-simulation-and-control-part-1-setting-up-the-control-problem-1539323440930.html)
- [ ] [Introduction to 6-DOF Simulation of Air Vehicles (pdf)](http://avionics.nau.edu.ua/files/doc/VisSim.doc/6dof.pdf)
- [ ] [Quadcopter Dynamics, Simulation and Control (paper)](http://andrew.gibiansky.com/downloads/pdf/Quadcopter%20Dynamics,%20Simulation,%20and%20Control.pdf)
- [ ] [PID, LQR and LQR-PID on a quadcopter platform (paper)](https://www.researchgate.net/publication/261212676_PID_LQR_and_LQR-PID_on_a_quadcopter_platform)

__At this point, you might wonder: This is only for four rotors. Don't worry, the extra ones are only used to have resilience. In the future this is going to be populated with more types of rotorcraft like submarines, VTOL drones and wings.__

</details>

## Control Theory

<details>
<summary>Control Theory</summary>

- Now that you have got a rough understanding of how you are going to make things fly, it's time to take a better look into the mathematical concepts behind the control of systems.

- [ ] [Matlab Tech Talks - State Space](https://www.mathworks.com/videos/series/state-space.html)
- [ ] [Matlab Tech Talks - Control Systems in practise](https://www.mathworks.com/videos/series/control-systems-in-practice.html)
- [ ] [Matlab Tech Talks - Understanding PID control](https://www.mathworks.com/videos/series/understanding-pid-control.html)

</details>

## Sensors and State Estimation

<details>
<summary>Sensors and State Estimation</summary>

- In order to get the control theory to work, we have to obtain a best-effort state estimate by observing our rotorcraft system.

- [ ] [Kalman Filters]
    - [ ] [Matlab Tech Talks - Understanding Kalman Filters](https://www.mathworks.com/videos/series/understanding-kalman-filters.html)
    - [ ] [(or) Michel van Biezen - Kalman Filters](https://www.youtube.com/watch?v=CaCcOwJPytQ)
- [ ] [Particle Filters](https://www.youtube.com/watch?v=lzN18y_z6HQ)
- [ ] [(optional) Michel van Biezen - How GPS works](https://www.youtube.com/watch?v=16xHIBmul_o&list=PLX2gX-ftPVXXGdn_8m2HCIJS7CfKMCwol)
- [ ] [Map Projections]
    - [ ] [Map Projections Explained](https://www.youtube.com/watch?v=wlfLW1j05Dg)
    - [ ] [Equirectangular Projection](https://en.wikipedia.org/wiki/Equirectangular_projection)
- [ ] [Inertia Measurement Units]
    - [ ] [How IMUs Work](https://www.youtube.com/watch?v=eqZgxR6eRjo)
    - [ ] [How to implement an IMU](https://www.youtube.com/watch?v=T9jXoG0QYIA)
    - [ ] [Soft mounting and vibrations](https://www.youtube.com/watch?v=zdE1BidMwNU)
- [ ] [Cameras and Optical Sensors]
    - [ ] [How does a camera work](https://www.youtube.com/watch?v=qS1FmgPVLqw)
    - [ ] [Rolling and Global Shutter](https://www.youtube.com/watch?v=DG4OjpD3Zow)
    - [ ] [Optical Flow]
        - [ ] [Computerphile - Optical Flow](https://www.youtube.com/watch?v=5AUypv5BNbI)
        - [ ] [Computerphile - Optical Flow Solutions](https://www.youtube.com/watch?v=4v_keMNROv4)
    - [ ] [Depth Perception]
        - [How Kinect works in 2 minutes](https://www.youtube.com/watch?v=uq9SEJxZiUg)
        - [Stereoscopic 3D basics](https://www.youtube.com/watch?v=1MXNRrHLuWk)
        - [How does a LiDAR work](https://www.youtube.com/watch?v=EYbhNSUnIdU)
        - [The correspondence problem](https://www.youtube.com/watch?v=VZNN1OGoqr8)]
- [ ] [Bias Correction and Calibration]
    - [ ] [WIP](https://example.com)
- [ ] [Sampling Techniques]
    - [ ] [WIP](https://example.com)
- [ ] [Communication]
    - [ ] [MCU]
        - [ ] [WIP](https://example.com)
    - [ ] [Remote]
        - [ ] [WIP](https://example.com)

</details>

## SLAM

<details>
<summary>Simultaneous Localization and Mapping (henceforth: SLAM)</summary>

</details>

## Path Planning

<details>
<summary></summary>

</details>

## Mechanical Parts

<details>
<summary>Motors, Propellers and other mechanical concepts</summary>

</details>


## The Flight Controller

<details>
<summary></summary>

</details>

## Existing Drone Software

<details>
<summary></summary>

</details>

## Existing Drone Hardware

<details>
<summary></summary>

</details>

## Building a Racing FPV Quadcopter

<details>
<summary></summary>

</details>

## Building Fully Autonomous Rotorcraft

<details>
<summary></summary>

</details>

## Once You've Finished

Congratulations!

Keep learning.

You're never really done.
