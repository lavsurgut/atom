from twisted.internet.protocol import Protocol

class USBRouterProtocol(Protocol):
    def __init__(self):
        self.debug = 0
        self.lenMsg = 38
        self.msg = []
        self.readyMsg = []
        #   self.destClients = []
        self.clients = []

    # def setDestClients(self, destClients):
    #   self.destClients = destClients
    #    print('added destinaton clients')

    def getClients(self):
        return self.clients

    def connectionFailed(self):
        print("Connection Failed:", self)
        # reactor.stop()

    def connectionMade(self):
        print('Serial port connected.')
        self.clients.append(self)

    def getMsg(self):
        return self.readyMsg

    def parseMsg(self, value, lenMsg):
        """ Check if received value is correct
            refill the array once the start symbol received(77)
            send the array to clients once the end symbol received(69)
            only message minimum of 38 values is being sent
            also escape character (120) is checked and removed where needed """
        try:
            lastValue = self.msg[-1]
        except IndexError:
            lastValue = -1

        if value == 77 and lastValue != 120:
            del self.msg[:]

        elif value == 69 and lastValue != 120:
            if len(self.msg) >= lenMsg:
                print("Message received: " + str(self.msg))
                self.readyMsg = list(self.msg)
            else:
                del self.msg[:]
        else:
            if value == 77 or value == 69:
                # remove escape character from start and end flag symbols, if they came in the body
                # because in this case they are ingenious data in the body, not the flags
                self.msg.pop()
                # add data to the array
                self.msg.append(value)
            else:
                self.msg.append(value)

    def validateDebugMsg(self, data):
        lenMsg = 1
        print(data)

    def validateMsg(self, value):
        lenMsg = 38
        self.parseMsg(value, lenMsg)

    def dataReceived(self, data):
        value = int.from_bytes(data, byteorder='little')
        if self.debug == 1:
            self.validateDebugMsg(data)
        else:
            self.validateMsg(value)

    def onClose(self):
        if self in self.clients:
            print("Removing " + str(self))
            self.clients.remove(self)
