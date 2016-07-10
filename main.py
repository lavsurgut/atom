from klein import Klein


from twisted.internet.serialport import SerialPort
from twisted.web.static import File

from atom.usb import USBRouterProtocol
from atom.ws import WSRouterFactory

from atom.web import MainElement

app = Klein()


@app.route('/')
def home(request, name=''):
    return MainElement(name)


@app.route('/static/', branch=True)
def static(request):
    return File("./static")


from txws import WebSocketFactory
from twisted.internet import reactor

wsFactory = WSRouterFactory()
usbClient = USBRouterProtocol()

reactor.listenTCP(5600, WebSocketFactory(wsFactory))

SerialPort(usbClient, '/dev/ttyUSB0', reactor, baudrate='9600')

wsFactory.setSrcClients(usbClient.getClients())

resource = app.resource
