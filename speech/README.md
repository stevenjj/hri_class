Installing Speech Recognition Libraries in ROS

October 13, 2016 | Adam Allevato
This post describes how to set up speech recognition to be integrated with ROS, using the Sphinx libraries and custom code developed by human-robot interaction researchers at UT Austin and elsewhere.

Platform: Ubuntu 14.04

ROS Version: Indigo

Download the CMU Sphinx library from this link. 
(https://sourceforge.net/projects/cmusphinx/files/sphinxbase/5prealpha/)

Choose the link for “sphinxbase-5-prealpha.tar.gz”, which is the Ubuntu source. Extract the folder into an existing catkin workspace, in a special subdirectory for speech, say, ~/catkin_ws/src/speech/sphinxbase.

In general, we will follow these instructions,
(https://github.com/cmusphinx/sphinxbase)

but we’re not using latest source, so our procedure is slightly different.
Enter your sphinxbase directory and run the following commands:

./autogen.sh
./configure
make
make check
sudo make install


There is a good chance that you’ll have to install some packages (e.g. sudo apt-get install autoconf bison swig, which is what I had to install) to perform the ./autogen.sh step.

Now we will follow similar steps for pocketsphinx. Download from this link. 
(https://sourceforge.net/projects/cmusphinx/files/pocketsphinx/5prealpha/)

Choose “pocketsphinx-5-prealpha.tar.gz.” Extract to ~/catkin_ws/src/speech/pocketsphinx.

Generally follow the directions here.
(https://github.com/cmusphinx/pocketsphinx%22)


Run these commands in the pocketsphinx directory:

./autogen.sh
./configure
make
make check
sudo make install
Now we will install the HLP-R package. Go up one directory to ~/catkin_ws/src/speech and execute the following command:

git clone https://github.com/HLP-R/hlpr_speech.git
This will create the hlpr_speech directory, which contains a catkin package.

catkin_make your workspace.

Source your workspace if you haven’t already.

Set up your speech dictionary using CMU’s lmtool.
(http://www.speech.cs.cmu.edu/tools/lmtool-new.html)

Go to the path ~/catkin_ws/src/speech/hlpr_speech/hlpr_speech_recognition/data and modify the files you find there. You can make new files if you wish, but you will have to modify the launch files to point to them. It’s easiest to just change the existing files.
Open kps.txt and put the commands you want to recognize in the file, one per line.
Save and upload kps.txt to the lmtool (link above), and press the COMPILE KNOWLEDGE BASE button.
After the files have been generated, download the XXXX.dic and XXXX.lm files. Rename them kps.dic and kps.lm, overwriting the existing files. You will have to build your own kps.yaml, using the existing file as a guide. kps.map is only necessary if you want to use the GUI (see below). If you need it, just copy the syntax of the existing .map file.
You are now ready to run speech recognition! There are two launch files included with HLP-R, but you should only ever use one of them:, speech_rec.launch, is what you will want to use for actual listening. It publishes detected speech commands to a ROS topic.

By default, the node will launch a GUI that allows voice commands to be simulated by clicking buttons. If you don’t need the gui, you can disable it with the parameter specified below. If you’re not using the GUI, you also don’t need to make a .map file, as the only purpose of the .map file is to determing the GUI buttons. To run the speech listener, run

   roslaunch hlpr_speech_recognition speech_rec.launch speech_gui:=false
You may have an error because you are missing pyaudio. If so, simply sudo apt-get install python-pyaudio and try again.

Happy speaking!


I had to do the extra steps as well
 1177  sudo apt-get install python-pyaudio
 1235  sudo apt-get install pulseaudio
 1236  sudo apt-get install libpulse-dev
 1237  sudo apt-get install osspd
 1245  sudo apt-get install espeak
 1246  sudo apt-get install python-espeak
