"""pyExitEmitter controller."""
import sys
from controller import Supervisor, Emitter

TIME_STEP = 256

supervisor = Supervisor()
emitter = supervisor.getEmitter("exitEmitter")
emitter.setChannel(2)
emitter.setRange(0.1)

counter = 0
noEntrance = (162,"exit")

while supervisor.step(TIME_STEP != -1):
    if(counter >= 16):
        emitter.send(noEntrance)
        counter = 0
        print(counter)
    else:
        counter+=1