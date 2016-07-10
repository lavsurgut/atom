from twisted.internet.protocol import Protocol, Factory
from twisted.internet import task
import json

class WSRouterProtocol(Protocol):
    def __init__(self, factory):
        self.factory = factory

    def connectionMade(self):
        self.factory.clientConnectionMade(self)
        #for cli in self.factory.destClients:
        #    cli.setDestClients(self.factory.clients)

    def dataReceived(self, data):
            # self.transport.write(data)

        print("Text message received: {0}".format(data.decode('utf8')))
        line = format(data.decode('utf8'))
            # # write into serial clients
        for cli in self.factory.srcClients:
            #     # for c in line:
            cli.transport.write(bytes([int(line)]))

    def connectionLost(self, reason):
        self.factory.clientConnectionLost(self)
        #for cli in self.factory.destClients:
        #    cli.setDestClients(self.factory.clients)


class WSRouterFactory(Factory):
    protocol = WSRouterProtocol

    def __init__(self):
        self.clients = []
        self.srcClients = []
        self.lc = task.LoopingCall(self.pullSerialData)
        self.lc.start(1)

    def pullSerialData(self):
        for src in self.srcClients:
            data = src.getMsg()
            print('Sending message to UI: ' + str(data))
            payload = json.dumps(data, ensure_ascii=False).encode('utf8')
            for client in self.clients:
                client.transport.write(payload)

    def buildProtocol(self, addr):
        return WSRouterProtocol(self)

    def getClients(self):
        return self.clients

    def clientConnectionMade(self, client):
        self.clients.append(client)

    def clientConnectionLost(self, client):
        self.clients.remove(client)

    def setSrcClients(self, srcClients):
        self.srcClients = srcClients
