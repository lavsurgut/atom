
var json;

var wsUri = "ws://localhost:5600/";
var websocket;
var command = 0;
var speed = 128;

function dec2bin(dec){
    return (dec >>> 0).toString(2);
}

function pad(n, width, z) {
  z = z || '0';
  n = n + '';
  return n.length >= width ? n : new Array(width - n.length + 1).join(z) + n;
}

function sleep(milliseconds) {
  var start = new Date().getTime();
  for (var i = 0; i < 1e7; i++) {
    if ((new Date().getTime() - start) > milliseconds){
      break;
    }
  }
}

function formFloat(intPartH, intPartL, decPartL) {
    hbyte = dec2bin(parseInt(intPartH, 10));
    lbyte = dec2bin(parseInt(intPartL, 10));
    deg = (hbyte << 8) | parseInt(lbyte, 2);
    decimPart = parseInt(decPartL, 10);
    //alert(deg);
    if ((deg >= 0) && (deg <= 359)) {
        float = deg + (decimPart*0.01);
    }
    return float;
}

function formCommand()
{

   document.getElementById("outputCommand").innerHTML = "COMMAND: " + pad(command.toString(2), 8);
}

function formSpeed()
{

   document.getElementById("outputSpeed").innerHTML = "SPEED: " + pad(speed.toString(2), 8);
}



function writeToScreen(message)
{
    document.getElementById("outputLog").innerHTML = message;
}




jQuery(document).ready(function ($) {
    window.chat = {};
    chat.ws = $.gracefulWebSocket(wsUri);
    writeToScreen("CONNECTED");

    chat.send = function (message) {
        chat.ws.send(message);
    };

    chat.ws.onerror = function (event) {
        writeToScreen("DISCONNECTED");
    };

    chat.ws.onclose = function (event) {
        writeToScreen("DISCONNECTED");
    };

    chat.ws.onmessage = function (event) {
         json = JSON.parse(event.data);

        gg1.refresh((json[0] != null) ? json[0]:0);
        gg2.refresh((json[1] != null) ? json[1]:0);
        gg3.refresh((json[2] != null) ? json[2]:0);

        gg4.refresh((json[15] != null) ? json[15]:0);
        gg5.refresh((json[16] != null) ? json[16]:0);
    };

    var btnFwd = document.getElementById("btn_fwd");
    var btnStp = document.getElementById("btn_stp");
    var btnBck = document.getElementById("btn_bck");
    var btnLft = document.getElementById("btn_lft");
    var btnRht = document.getElementById("btn_rht");
    var btnLog = document.getElementById("btn_log");
    var btnRde = document.getElementById("btn_rde");

    var lft_1 = document.getElementById("lft_1");
    var lft_2 = document.getElementById("lft_2");
    var lft_3 = document.getElementById("lft_3");
    var lft_4 = document.getElementById("lft_4");

    var rht_1 = document.getElementById("rht_1");
    var rht_2 = document.getElementById("rht_2");
    var rht_3 = document.getElementById("rht_3");
    var rht_4 = document.getElementById("rht_4");

//    var textBox = document.getElementById("input_angle");
//
    btnFwd.addEventListener("click", function(e) {
      if (!e) { var e = window.event; }

        command = command | (1 << 3);
        command = command & ~(1 << 2);
        formCommand();
        chat.send(command);
        writeToScreen("SENT");

    }, false);

    btnStp.addEventListener("click", function(e) {
      if (!e) { var e = window.event; }

        command = command & ~(1 << 4) & ~(1 << 3) & ~(1 << 2) & ~(1 << 1) & ~(1 << 0)
        formCommand();
        chat.send(command);
        writeToScreen("SENT");

    }, false);


    btnLft.addEventListener("click", function(e) {
      if (!e) { var e = window.event; }

        command = command | (1 << 0)
        formCommand();
        chat.send(command);
        writeToScreen("SENT");

    }, false);


    btnRht.addEventListener("click", function(e) {
      if (!e) { var e = window.event; }

        command = command | (1 << 1)
        formCommand();
        chat.send(command);
        writeToScreen("SENT");

    }, false);


    btnBck.addEventListener("click", function(e) {
      if (!e) { var e = window.event; }

        command = command & ~(1 << 3)
        command = command | (1 << 2)
        formCommand();
        chat.send(command);
        writeToScreen("SENT");

    }, false);


    btnLog.addEventListener("click", function(e) {
      if (!e) { var e = window.event; }

        command = command | (1 << 4)
        formCommand()
        chat.send(command)
        command = command & ~ (1 << 4)
        writeToScreen("SENT");

    }, false);


    btnRde.addEventListener("click", function(e) {
      if (!e) { var e = window.event; }

        command = command | (1 << 5)
        formCommand()
        chat.send(command)
        command = command & ~ (1 << 5)
        writeToScreen("SENT");

    }, false);

    lft_1.addEventListener("click", function(e) {
      if (!e) { var e = window.event; }
        speed = speed & ~(1 << 3) & ~(1 << 2)
        formSpeed();
        chat.send(speed);
        writeToScreen("SENT");

    }, false);

    lft_2.addEventListener("click", function(e) {
      if (!e) { var e = window.event; }
        speed = speed & ~(1 << 3)
        speed = speed | (1 << 2)
        formSpeed();
        chat.send(speed);
        writeToScreen("SENT");

    }, false);

    lft_3.addEventListener("click", function(e) {
      if (!e) { var e = window.event; }
        speed = speed & ~(1 << 2)
        speed = speed | (1 << 3)
        formSpeed();
        chat.send(speed);
        writeToScreen("SENT");

    }, false);

    lft_4.addEventListener("click", function(e) {
      if (!e) { var e = window.event; }

        speed = speed | (1 << 3)| (1 << 2)
        formSpeed();
        chat.send(speed);
        writeToScreen("SENT");

    }, false);

    rht_1.addEventListener("click", function(e) {
      if (!e) { var e = window.event; }
        speed = speed & ~(1 << 0) & ~(1 << 1)
        formSpeed();
        chat.send(speed);
        writeToScreen("SENT");

    }, false);

    rht_2.addEventListener("click", function(e) {
      if (!e) { var e = window.event; }
        speed = speed & ~(1 << 1)
        speed = speed | (1 << 0)
        formSpeed();
        chat.send(speed);
        writeToScreen("SENT");

    }, false);

    rht_3.addEventListener("click", function(e) {
      if (!e) { var e = window.event; }
        speed = speed & ~(1 << 0)
        speed = speed | (1 << 1)
        formSpeed();
        chat.send(speed);
        writeToScreen("SENT");

    }, false);

    rht_4.addEventListener("click", function(e) {
      if (!e) { var e = window.event; }

        speed = speed | (1 << 0)| (1 << 1)
        formSpeed();
        chat.send(speed);
        writeToScreen("SENT");

    }, false);


    var dflt = {
      min: 0,
      max: 200,
      donut: true,
      gaugeWidthScale: 0.4,
      counter: true,
      value: 0,
      titleFontFamily: 'DeJaVu Sans Mono',
      hideInnerShadow: true
    };

    var gg1 = new JustGage({
      id: 'gg1',
      title: 'Temperature',
      defaults: dflt
    });

    var gg2 = new JustGage({
      id: 'gg2',
      title: 'Altitude',
      defaults: dflt
    });

    var gg3 = new JustGage({
      id: 'gg3',
      title: 'Acceleration',
      defaults: dflt
    });

    var gg4 = new JustGage({
      id: 'gg4',
      title: 'Left wheel speed',
      defaults: dflt,
      hideInnerShadow: false,
      levelColors: [
          "#00fff6",
          "#ff00fc",
          "#1200ff"
        ]
    });

    var gg5 = new JustGage({
      id: 'gg5',
      title: 'Right wheel speed',
      defaults: dflt,
      hideInnerShadow: false,
      levelColors: [
          "#00fff6",
          "#ff00fc",
          "#1200ff"
        ]
    });


});
