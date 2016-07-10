from twisted.web.template import Element, renderer, XMLFile
from twisted.python.filepath import FilePath

class MainElement(Element):
    loader = XMLFile(FilePath('templates/template.xml'))

    def __init__(self, name):
        self._name = name

    @renderer
    def name(self, request, tag):
        return self._name