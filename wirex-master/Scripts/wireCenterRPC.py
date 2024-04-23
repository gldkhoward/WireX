# 
# WireX
#
# Copyright (c) 2006-2019 Andreas Pott
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.
# 
"""
Created on 11.11.2013

@author: plm
"""

import socket
import struct
import time
from time import sleep
import marshal


socketObj = 0
#    BUFFER_SIZE = 1024

def connect(ip, port):
    global socketObj
    socketObj = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    print "Try to connect to " + str(ip) + ":" + str(port)
    socketObj.connect((ip, port))
    # set socket timeout
    socketObj.settimeout(2) # [seconds]
    sleep(5)
    byteArray = socketObj.recv(1)
    response_int8 = struct.unpack('b',byteArray)[0] # b: signed char = 1byte integer
    
    if response_int8 == -1: # server is busy
        print('Server is already processing messages.\n')
        socketObj.close()
        print('Try again later... connection closed.\n\n')
    elif response_int8 == 1: # server is ready to receive messages
        print('Connected to server.\n')
    else:
        print('Unrecognized response message: ' + str(response_int8) +'\n')
        socketObj.close()
        print('Connection closed.\n\n');
         

def sendMessage(pyString):
    flag = 2 # only python marshaling can be called
    flag_byte = struct.pack('i',flag)
    
    pyString_byte = struct.pack(str(len(pyString))+'s', pyString)
    pyStringLength = len(pyString_byte)
    pyStringLength_byte = struct.pack('i', pyStringLength)
    msgLength = pyStringLength + 4 + 4 # 4byte flagSize + 4byte pyStringSize value
    msgLength_byte = struct.pack('i',msgLength)
    
    bufSend = bytearray(msgLength)
    bufSend[0:4] = msgLength_byte
    bufSend[4:8] = flag_byte
    bufSend[8:12] = pyStringLength_byte
    bufSend[12:] = pyString_byte
    
    socketObj.send(bufSend)
    
def readResponseMessage():
    bytesRead = 0 # indicated how many bytes were read
    msg_len_byte = ''
    timeout_counter = 0
    #try:
    while (bytesRead != 4 and timeout_counter < 100):
        msg_len_byte = msg_len_byte + socketObj.recv(4-bytesRead) # read message length
        bytesRead = len(msg_len_byte)

    # convert to uint32
    msg_len_int32 = struct.unpack('i',msg_len_byte)[0]
    
    # read message (msg_len_uint32 - 4) bytes
    bufDataIn = ''
    bytesRead = 0
    timeout_counter = 0
    while (bytesRead != msg_len_int32 and timeout_counter < 100):
        bufDataIn = bufDataIn + socketObj.recv(msg_len_int32-bytesRead)
        bytesRead = len(bufDataIn)
        
    # get serialized object string
    msgString = struct.unpack(str(msg_len_int32)+'s',bufDataIn)[0]
    
    return msgString

def closeConnection():
    socketObj.close()
    print "Connection closed"

def rpc(*arg):
    m = marshal.dumps(arg)
    sendMessage(m)
    msgString = readResponseMessage()
#   print(msgString)
#   print msgString.encode("hex")
    m_out = marshal.loads(msgString)
    return m_out


def main():
    
    global socketObj
    
    ip = '127.0.0.1'
    port = 9050
    connect(ip, port)
    
    v=(1,1,1)
    x = rpc('Irobot.setPosition', v[0], v[1], v[2])
    x = rpc('Iapp.createBox', 0,0,0,1.1,1.1,1.1)
    time.clock()
    counter = 0
    while (time.clock() < 1):
        x = rpc('Irobot.inverseKinematics',0,0.0,0.0,0.0)
        counter = counter + 1
    print 'Inverse kinematics result: ' + str(x)
    print str(counter) + ' calls per second'
    
    
    closeConnection()
    
    
if __name__ == "__main__":
    main()
