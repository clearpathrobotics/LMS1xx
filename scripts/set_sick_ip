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

#Use: rosrun lms1xx set_sick_ip <MAC_ADDR> <DESIRED_IP_ADDR>
#e.g. rosrun lms1xx set_sick_ip 000677206c35 192.168.1.17

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

try:
    sys.argv[1]
    sys.argv[2]
except IndexError:
    print "Not enough arguments."
    print "Use: rosrun lms1xx set_sick_ip <MAC_ADDR> <DESIRED_IP_ADDR>"
    sys.exit()

#set variables
STATIC_IP = str(sys.argv[2])
MAC_ADDR = str(sys.argv[1])
MAC_ADDR_FORMATTED = ':'.join(MAC_ADDR[i:i+2] for i in range(0, len(MAC_ADDR), 2))

UDP_PORT = 30718

UDP_DATA_discovery = '10000008ffffffffffff412eb5ee01000ad33705ffffff00'.decode('hex')

BINARY_HEADER = '11000101' + MAC_ADDR + '03da4af00100'
SET_IP_XML =  '''\
<?xml version="1.0" encoding="UTF-8"?>
<IPconfig MACAddr="''' + MAC_ADDR_FORMATTED + '''">
<Item key="IPMask" value="255.255.255.0"/>
<Item key="DHCPClientEnabled" value="FALSE"/>
<Item key="IPGateway" value="0.0.0.0"/>
<Item key="IPAddress" value="''' + STATIC_IP + '"/></IPconfig>'
UDP_DATA_setIP = BINARY_HEADER.decode('hex') + SET_IP_XML

parser = MyHTMLParser()
LASER_NOT_FOUND  = "Laser with MAC address '" + MAC_ADDR_FORMATTED + "' not found.\n"

#open UDP socket
UDP_SOCK = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
UDP_SOCK.settimeout(2)
UDP_SOCK.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)

#send discovery cmd
UDP_SOCK.sendto(UDP_DATA_discovery, ('<broadcast>', UDP_PORT))
try:
    #read reply to discovery cmd
    data = ""
    data,addr = UDP_SOCK.recvfrom(1024)

    if ('<?xml' in data):
        UDP_SOCK.sendto(UDP_DATA_setIP, ('<broadcast>', UDP_PORT))
        try:
            #read reply to set IP cmd
            data,addr = UDP_SOCK.recvfrom(1024)

            if ('<?xml' in data and 'Success' in data):
                print "Laser found, IP address set succesfully:"
                parser.feed(data)
                #set IP cmd response from laser doesn't contain the new IP addr,
                #hence the below print statement.
                print "   " + STATIC_IP + "\n"
        except socket.timeout:
            print LASER_NOT_FOUND
    else:
        print LASER_NOT_FOUND

except socket.timeout:
    print LASER_NOT_FOUND

UDP_SOCK.close()
