from twisted.internet.protocol import Protocol, Factory


class EchoProtocol(Protocol):
    def dataReceived(self, data):
        self.transport.write(data)


class EchoFactory(Factory):
    protocol = EchoProtocol
