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

## Run an example to check if voice recognition works 

Run the testing script:

```
rosrun demo_voice_control test_voice_control_node.py
```

Speak one of the default commands ( start / halt / open / close / robot )

You can see the messages of the words you spoke in:
```

```

## Run Teaching script with Robotiq gripper 


## Using your own keywords

You can run this with any set of words. To do that, you need lexicon and keyword list files
(check voice_cmd.dic and voice_cmd.kwlist for details). 

Word pronunciations for English can be found in 
[CMUdict](https://sourceforge.net/projects/cmusphinx/files/G2P%20Models/phonetisaurus-cmudict-split.tar.gz)

You can also download pocketsphinx acoustic models for several other languages [here](https://sourceforge.net/projects/cmusphinx/files/)

Read more about pocketsphinx on the official website: http://cmusphinx.sourceforge.net
