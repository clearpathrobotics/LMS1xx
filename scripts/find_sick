#!/usr/bin/env python
# Software License Agreement (BSD)
#
# @author    Mustafa Safri <msafri@clearpathrobotics.com>
# @copyright (c) 2015, Clearpath Robotics, Inc., All rights reserved.
#
# Redistribution and use in source and binary forms, with or without modification, are permitted provided that
# the following conditions are met:
# * Redistributions of source code must retain the above copyright notice, this list of conditions and the
#   following disclaimer.
# * Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the
#   following disclaimer in the documentation and/or other materials provided with the distribution.
# * Neither the name of Clearpath Robotics nor the names of its contributors may be used to endorse or
#   promote products derived from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED
# WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
# PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
# ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
# TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
# HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
# NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

#Use: rosrun lms1xx find_sick

import sys
import socket
from HTMLParser import HTMLParser

class MyHTMLParser(HTMLParser):
    def handle_starttag(self, tag, attrs):
        attr_list = []
        for attr in attrs:
            attr_list.append(attr[1])
        if (len(attr_list) == 1):
            print "  ", attr_list[0],
        elif ('IPAddress' in attr_list[0]):
            print "   ", attr_list[1]

#set variables
UDP_PORT = 30718
UDP_DATA_discovery = '10000008ffffffffffff412eb5ee01000ad33705ffffff00'.decode('hex')

parser = MyHTMLParser()
data = ""
laser_count = 0

#open UDP socket
UDP_SOCK = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
UDP_SOCK.settimeout(2)
UDP_SOCK.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)

#send UDP broadcast
UDP_SOCK.sendto(UDP_DATA_discovery, ('<broadcast>', UDP_PORT))

#receive UDP reply
while True:
    try:
        data,addr = UDP_SOCK.recvfrom(1024)
        if ('<?xml' in data):
            laser_count += 1
            if (laser_count == 1):
                print "Lasers found:"
            parser.feed(data)
    except socket.timeout:
        break;

if (laser_count == 0):
    print "No lasers found.\n"
else:
    print ""

UDP_SOCK.close()
