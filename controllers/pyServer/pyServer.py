"""pyServer controller."""

from controller import Supervisor, Receiver

TIME_STEP = 256

supervisor = Supervisor()
receiver = supervisor.getReceiver("receiver")
receiver.enable(TIME_STEP)
receiver.setChannel(-1)

while supervisor.step(TIME_STEP) != -1:
    print('in while loop')
    if(receiver.getQueueLength() > 0):
        incoming = receiver.getData()
        print('incoming')
        dataList = struct.unpack('d',incoming)
        print(dataList)
        receiver.nextPacket()

