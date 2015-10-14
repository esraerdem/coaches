Python GUI

- install Python 2.7
- install pip (for future use) - sudo apt-get install python-pip
- install TkImage - sudo apt-get install python-imaging-tk


To test the GUI independently of the rest of the COACHES software:

- Run the tcp_interface node: 
>> roscore&
>> rosrun tcp_interface tcp_interface

- Run the GUI
>> python QAGUI.py

- Send commands to the GUI through the tcp_interface using rostopic pub:
>> rostopic pub -1 /RCOMMessage tcp_interface/RCOMMessage "header:
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: ''
robotsender: 'NONAME'
robotreceiver: '127.0.0.1:41809'
value: 'display_image_toilets'"

robotreceiver must contain the IP and port of the GUI. To retrieve this information you can
consult the output of the tcp_interface node.

value contains the instruction. By the moment, display_text_welcome and display_image_toilets are supported.
