from twisted.internet.protocol import Protocol
import json

class USBClient(Protocol):
    # debug mode
    debug = 0
    # length of the package
    lenMsg = 38

    msg = []

    def __init__(self, debug=False):
        self.debug = debug

    def connectionFailed(self):
        print("Connection Failed:", self)
        # reactor.stop()

    def connectionMade(self):
        print('Serial port connected.')

    # Method to show the data or send it to the UI
    # You can use your own code here to display it in preferred manner
    def dataReceived(self, data):
        pass
        value = int.from_bytes(data, byteorder='little')
        if self.debug == 1:
            lenMsg = 1
        else:
            lenMsg = 38
            # print(repr(data))

        # Check if received value is correct
        # refill the array once the start symbol received(77)serial.serialutil.SerialException
        # send the array to clients once the end symbol received(69)
        # only message minimum of 38 values is being sent
        # also escape character (120) is checked and removed where needed
        if value >= 0 and value <= 255:
            try:
                lastValue = self.msg[-1]
            except IndexError:
                lastValue = -1

            if value == 77 and lastValue != 120:
                del self.msg[:]

            elif value == 69 and lastValue != 120:
                if self.debug == 1:
                    print(str(self.msg))
                else:
                    if len(self.msg) >= lenMsg:
                        print("Sending message to clients: " + str(self.msg))
                        # package data into json and send
                        payload = json.dumps(self.msg, ensure_ascii=False).encode('utf8')
                        # write into web clients
                        # for cli in client_list:
                         #   cli.sendMessage(payload, False)
                    else:
                        del self.msg[:]
            else:
                if value == 77 or value == 69:
                    # remove escape character from start and end flag symbols, if they came in the body
                    # because in this case they are ingenious data in the body, not the flags
                    self.msg.pop()
                    # add data to the array
                    if self.debug == 1:
                        self.msg.append(data)
                    else:
                        self.msg.append(value)
                else:
                    if self.debug == 1:
                        self.msg.append(data)
                    else:
                        self.msg.append(value)

    # write into serial
    def sendLine(self, cmd):
        print(cmd)
        self.transport.write(cmd)
