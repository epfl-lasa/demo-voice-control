## Voice control for Easy Kinesthetic Teaching with pocketsphinx

Code adapted from https://github.com/gorinars/ros_voice_control.git

The script shows how to control a gripper and start/stop measurement recordings with English keywords using pocketsphinx

## Installation

### Install pocketsphinx with dependencies
```
sudo apt-get install -y python python-dev python-pip build-essential swig libpulse-dev git
sudo apt-get install libasound-dev
sudo apt-get install libasound2-dev
sudo apt-get install python-pyaudio
sudo pip install pocketsphinx
```

### Install language models
```
sudo apt-get install pocketsphinx-hmm-en-*
```

### Run simple voice recognition example 
The simple voicecontrol_class.py implements the Voice Recognition system for the dictionary defined in 'demo-voice-control/commands/
voice_command_test.dic', once word is recognized it publishes them as a string message. To test run the following script:

```
rosrun demo_voice_control test_voice_control_node.py
```
Speak one of the default commands ( back / forward / left / move / right / stop )

You can see the messages of the words you spoke in:
```
rostopic echo /demo_voice_control/command
```
### Using your own keywords

You can run this with any set of words. To do that, you need lexicon and keyword list files
(check voice_cmd.dic and voice_cmd.kwlist for details). 

Word pronunciations for English can be found in 
[CMUdict](https://sourceforge.net/projects/cmusphinx/files/G2P%20Models/phonetisaurus-cmudict-split.tar.gz)

You can also download pocketsphinx acoustic models for several other languages [here](https://sourceforge.net/projects/cmusphinx/files/)

Read more about pocketsphinx on the official website: http://cmusphinx.sourceforge.net


## Application: Voice-control of Robotiq gripper + Data Recording

### Dependencies
First you must install the [Robotiq](http://wiki.ros.org/robotiq) gripper controller and dependencies, follow the instuctions here: https://github.com/epfl-lasa/lasa-wiki/wiki/Robotiq-gripper

### Examples
The teach_voicecontrol_class.py implements the Voice Recognition system for the dictionary defined in 'demo-voice-control/commands/
voice_command.dic' to open/close the gripper with voice activation and start/stop data recording.  

- To use only the gripper command, run the following launch file:
```
roslaunch demo_voice_control teach_voice_control.launch
```
If you speak one of the default commands ( open / close ) the gripper should open / close.

- **[TODO]** To run full teaching commands with voice acivation, run the following launch file:
```
roslaunch demo_voice_control teach_voice_control.launch record:=true ...
```
If you speak one of the default commands ( open / close / start / halt ) the gripper should open / close and data recordings will start / halt.
