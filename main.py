from klein import Klein

from serial.serialutil import SerialException

from twisted.internet.serialport import SerialPort
from twisted.web.static import File

from atom.serial import USBClient
from atom.ws import EchoFactory

from atom.web import HelloElement

app = Klein()


@app.route('/')
def home(request, name='world'):
    return HelloElement(name)


@app.route('/static/', branch=True)
def static(request):
    return File("./static")


from txws import WebSocketFactory
from twisted.internet import reactor

reactor.listenTCP(5600, WebSocketFactory(EchoFactory()))

try:
    SerialPort(USBClient(), '/dev/ttyUSB0', reactor, baudrate='9600')
except SerialException:
    print('Connection failed.')

resource = app.resource
