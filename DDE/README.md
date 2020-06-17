# DDE / Job Engine

Dexter Development Environment is the IDE for Dexter. It is written in javascript and runs as an electron app on the desktop, and under node.js on Dexter. When used to run DDE programs (aka "jobs") on Dexter, it is known as the "Job Engine"

The current long term release version of DDE, which will not change as rapidly as the development version, is avaiable at:<br>
https://github.com/cfry/dde/releases/tag/v2.5.13

Newer development releases of DDE can be found on the [releases tab](https://github.com/cfry/dde/tags) at:<br>
https://github.com/cfry/dde

BETA version [3.0.7](https://github.com/cfry/dde/releases/tag/v3.0.7) is known to passes most tests on the internal test suite, it can move the robot in open loop modes, and the calibration system works well for closed loop modes. It is the version used on Dexter as the Job Engine, but this will change in the future as newer versions pass additional testing.

The current DDE documentation (precedded by some previously written articles) is available online here:
http://hdrobotic.com/software/

A series of webinars on the use of DDE is available here: 
http://hdrobotic.com/webinar (scroll down)

* [Intro to DDE](https://drive.google.com/open?id=0B6PCkmO9RJLJNnZRdkNkTDBoWTg)
Recorded: August 15, 2017  
Dexter Development Environment has an extensive help system. This Webinar describes the numerous facits of that help system so that you can teach yourself how to program in DDE.

* [Making Jobs](https://drive.google.com/open?id=0B6PCkmO9RJLJUzZJbGJZVGtHb3c)
Recorded: August 29, 2017  
A Job is the primary way to describe a process for building something. This webinar covers basic job building and running.

* [Human Instructions](https://drive.google.com/open?id=0B6PCkmO9RJLJdjBIQWUzRWVXbk0)
Recorded: September 5, 2017  
The core of a Job is its do_list of instructions. Some instructions are not meant to be carried out by a machine but rather its human operator. By packing these instructions just like machine instructions, we can more easily build processes where humans and machines cooperate to build something. This webinar tells you how to include such human instructions into a Job.

* [How to Think Like a Computer](https://youtu.be/_cje_ELEeR0)
Recorded:September 12, 2017  
Central to constructing working code is understanding how the computer interprets your code. `eval` is the function in JavaScript that accepts a string of source code and "runs" it. By running "eval" in your head as you are reading code, you can know what the computer will do woth it. This is the core technique of understanding and debugging JavaScript source code. The best way to know what eval does is to know how to write eval. This webinar helps you understand how eval works by incrementally defining it in JavaScript.

* [Making Music](https://drive.google.com/open?id=0B6PCkmO9RJLJU2VUVkhReGd6RjA)
Recorded: October 2, 2017  
DDE can send and receive MIDI events. To facilitate the sending of events, DDE has the Note class and a higher level Phrase class that have a number of "phrase processors" useful for constructing higher level scores based on combining Notes and Phrases into songs. DDE can also accept input from MIDI devices to control other processes.

* [show_window (and friends)](https://drive.google.com/open?id=0B6PCkmO9RJLJdnRQM2Q2b1F6Wm8)
Recorded: October 10, 2017  
DDE has a window system for helping you to construction Graphical User Interfaces (GUIs). This helps you connect presentation of the user interface and the values input by a user to other functions. The primary method to assist this is show_window. The show_window can be used outside of a Job. Human.show_window can be used within a Job like the other human instructions.

* [Debugging](https://drive.google.com/open?id=0B6PCkmO9RJLJTDFfYTNqYS1PdHM)
Recorded: October 17, 2017  
Debugging is time consuming, complex and necessary to get working programs. This webinar covers a multitude of techniques to get your programs (and Jobs) working.

* [Advanced Jobs](https://drive.google.com/file/d/0B6PCkmO9RJLJZ2RLN3VVWGdaakE/view)
Recorded: October 26, 2017  
Prerequisite: Webinar: Making Jobs.  
Covers the different kinds of instructions you can put on a do_list:
    *   Regular Instructions
    *   Function definition
    *   Arrays of instructions
    *   Human.enter_number (See Webinar: Human Instructions for depth)
    *   Generators
    *   Robot.sync_point
    *   Robot.send_to_job
    *   Robot.suspend
    *   Robot.go_to

    Covers Job initialization parameters:
    *   initial_instruction
    *   user_data
    *   when_stopped

* [Reliability via Testing](https://drive.google.com/open?id=0B6PCkmO9RJLJZVhwem1WZ0JoQzA)
Recorded: October 31, 2017  
Prerequisite: Knowing JavaScript syntax.  
Starts with a discussion of managing more complex programs and their bugs.  
Covers the details of using Dexter Development Environment's TestSuite system to help your code _stay_ working.

* [Machine Vision with OpenCV](https://drive.google.com/open?id=1n6lQmJDmPnbjSXzxRRz2pyHQxA-5_U-A)
Recorded: Nov 7, 2017  
Prerequisite: A bit of HTML and JavaScript.  
DDE's use of opencv.js is presented through the examples on DDE's Insert menu/Machine Vision including:
    *   color to black and white transformation
    *   blur an image
    *   blob detection for identifying objects
    *   webcam interface

* [How To Think Like A Job](https://drive.google.com/open?id=17rTFfH5aP-zFcG1oPjyfmh4Favi-AuVa)
Recorded: Nov 21, 2017  
Prerequisite: Watch webinar: _How To Think Like A Computer_  
A Job's eye view of controlling Dexter.  
Uses a personal "do list" as an analogy of what defining and running a Job is like.  
Covers:
    *   The digitization of _process_.
    *   Touches on serial vs parallel processing.
    *   The Job's _actor_ (a robot).
    *   Array of Instructions
    *   Start Objects
        *   Job
        *   TestSuite
        *   Note
        *   Phrase
        *   Custom Start Objects
    *   DDE's `Start Job` button.

