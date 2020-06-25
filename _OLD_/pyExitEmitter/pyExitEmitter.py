"""pyExitEmitter controller."""
import sys
from controller import Supervisor, Emitter
import struct

TIME_STEP = 256

supervisor = Supervisor()
emitter = supervisor.getEmitter("exitEmitter")
emitter.setChannel(2)
emitter.setRange(0.1)

counter = 0
noEntrance = struct.pack('is',162,b'exit')

#print("Before loop")

while supervisor.step(TIME_STEP) != -1:
#    print("before the if")
    if(counter >= 16):
        emitter.send(noEntrance)
        counter = 0
#        print(counter)
    else:
        counter+=1
#        print(counter)
        
#print("after loop")