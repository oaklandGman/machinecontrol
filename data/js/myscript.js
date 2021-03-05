$(document).ready(function() {
      var hostName = location.hostname;
      var socket;
      /*The user has WebSockets!!! */

      connect();

      function connect(){
          var host = "ws://"+hostName+"/ws";

          try{
              socket = new WebSocket(host); 
              socket.binaryType = "arraybuffer";

              $("#footerText").html("Connecting...");
              message('<ons-list-item class="event">Socket Status: '+socket.readyState);

              socket.onopen = function(){
             	 message('<ons-list-item class="event">Socket Status: '+socket.readyState+' (open)');
               $("#footerText").html("Connection Open");
              }

              socket.onmessage = function(msg) {
                if (msg.data instanceof ArrayBuffer) {
                  // binary frame
                  var myJson = unPack(msg.data);
                  // const view = new DataView(msg.data);
                  // console.log("Binary frame ", new Uint8Array(msg.data));
                  message('<ons-list-item class="message">Binary frame.');
                } else {
                  // text frame
                  var dataMsg = msg.data + '';
                  if (isJson(dataMsg)) {
                    var myJson = JSON.parse(dataMsg);
                    for (key in myJson) {
                      document.getElementById(key).value = myJson[key];
                    }
                  } else {
                    message('<ons-list-item class="message">Recv: '+dataMsg);
                  }
                }
              }
              

              socket.onclose = function(){
              	message('<ons-list-item class="event">Socket Status: '+socket.readyState+' (Closed)');
                 $("#footerText").html("Connection closed");
                 setTimeout(connect(), 2000);
              }			

          } catch(exception){
             message('<ons-list-item class="error">Error '+exception);
          }
      }//End connect

      $('.numbox').on("change", function(){
        var myId = $( this ).attr('id');
        var myValue = $( this ).val();
        var myMessage = {};
        
        myMessage.fnc = "motor";
        myMessage.cmd = myId;
        myMessage.dat = myValue;

        if (!isNaN(myValue)){
          sendPack(myMessage, socket);
        }
      });
  
      $('ons-switch').change(function(){
        var myId = $( this ).attr('id');
        var mySwitch = document.getElementById(myId);
        var myValue = mySwitch.checked;
        var myMessage ={};
        
        // myMessage.n = myName;

        if (myId=="motortest") {
          myMessage.fnc = "motor";
          myMessage.cmd = myId;
          if (myValue) { // checked
            myMessage.dat = 0x01;
          } else { // unchecked
            myMessage.dat = 0x00;
          }
        } else if (myId=="autolube") {
          myMessage.fnc = "motor"; 
          myMessage.cmd = myId; 
          if (myValue) { // checked
            myMessage.dat = 0x01;
          } else { // unchecked
            myMessage.dat = 0x00;
          }
        } else if (myId=="autostroke") {
          myMessage.fnc = "motor"; 
          myMessage.cmd = myId; 
          if (myValue) { // checked
            myMessage.dat = 0x01;
          } else { // unchecked
            myMessage.dat = 0x00;
          }

        }

        if (myMessage.dat != null){ // function, command, value
          sendPack(myMessage, socket);
        }
      }); 

      $('.button').click(function(){
        var myId = $( this ).attr('id');
        var myMessage ={};
        
        // myMessage.n = myName;

        myMessage.fnc = "motor";
        myMessage.cmd = myId;
        if        (myId=="calibrate") {
            myMessage.dat = 0x01;
        } else if (myId=="shootlube") {
            myMessage.dat = 0x01;
        } 

        if (myMessage.dat != null){ // function, command, value
          sendPack(myMessage, socket);
        }
      });
});

function sendPack(myMessage, socket) {
  var buffer = MessagePack.encode(myMessage);
  socket.send(buffer);
  // socket.send(JSON.stringify(myMessage));
  message('<ons-list-item class="action">Sent msgpack: ' + buffer.length + ' bytes.');
}

function message(msg){
  $('#diagList').prepend(msg+'</ons-list-item>');
  // ons.compile($('#diagList'));
}

function isJson(str) {
  try {
    JSON.parse(str);
  } catch (e) {
    return false;
  }
  return true;
}

