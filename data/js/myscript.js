ons.ready(function() {
  var hostName = location.hostname;
  var socket;
  /*The user has WebSockets!!! */

  document.addEventListener('prechange', function(event) {
    document.querySelector('ons-toolbar .center')
      .innerHTML = event.tabItem.getAttribute('label');
  });

  connect();

  function connect(){
      var host = "ws://"+hostName+"/ws";

      try{
          socket = new WebSocket(host); 
          socket.binaryType = "arraybuffer";

          $("#footerText").html("Connecting...");
          message('<ons-list-item class="event">Socket Status: '+socket.readyState);

          socket.onopen = function(){
            var myMessage={};
            myMessage.fnc="motor";
            myMessage.cmd="update";
            myMessage.dat=0x01;

            message('<ons-list-item class="event">Socket Status: '+socket.readyState+' (open)');
            $("#footerText").html("Connection Open");

            sendPack(myMessage, socket);

          }

          socket.onmessage = function(msg) {
            if (msg.data instanceof ArrayBuffer) {
              // binary frame
              var myJson = unPack(msg.data);
              // const view = new DataView(msg.data);
              // console.log("Binary frame ", new Uint8Array(msg.data));
              message('<ons-list-item class="message">Binary frame?');
            } else {
              // text frame
              var dataMsg = msg.data + '';
              // console.log(dataMsg);
              if (isJson(dataMsg)) {
                var myJson = JSON.parse(dataMsg);
                for (key in myJson) {
                  if (key != "switches") {
                    document.getElementById(key).value = myJson[key];
                  } else {
                    for ( sw in myJson.switches ) {
                      if (myJson.switches[sw]) {
                        document.getElementById(sw).checked = true;
                      } else {
                        document.getElementById(sw).checked = false;  
                      }
                    }
                  }
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


  var myTabbar = document.querySelector("ons-tabbar");
  myTabbar.addEventListener("postchange", function(e){
    // alert("tab " + e.index);
    $('.button').unbind().click(function( event ){ // listen for button click
      var myId = $( this ).attr('id'); 
      var myMessage ={};
      
      // myMessage.n = myName;
  
      myMessage.fnc = "motor";
      myMessage.cmd = myId;
      myMessage.dat = 0x01;
      if (myId=="info") {
          myMessage.fnc = "info";
          myMessage.dat = 0x01;
      }
  
      if (myMessage.dat != null){ // function, command, value
        sendPack(myMessage, socket);
      }
      event.preventDefault();
    }); // end button click

    $('ons-switch').unbind().change(function( event ){ // listen for switch change
      var myId = $( this ).attr('id');
      var mySwitch = document.getElementById(myId);
      var myValue = mySwitch.checked;
      var myMessage ={};
      
      // myMessage.n = myName;
  
      if (myId=="motenabled") {
        myMessage.fnc = "motor";
        myMessage.cmd = myId;
        if (myValue) { // checked
          myMessage.dat = 0x01;
        } else { // unchecked
          myMessage.dat = 0x00;
        }
      } else if (myId=="runtest") {
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
      // event.preventDefault();
    }); // end switch change event

    $('.numbox').unbind().on("change", function(){ // listen for numbox change
      var myId = $( this ).attr('id');
      var myValue = $( this ).val();
      var myMessage = {};
      
      myMessage.fnc = "motor";
      myMessage.cmd = myId;
      myMessage.dat = myValue;
  
      if (!isNaN(myValue)){
        sendPack(myMessage, socket);
      }
    }); // end numbox change  
  })
});


$(document).ready(function() {
});

function sendPack(myMessage, socket) {
  var buffer = MessagePack.encode(myMessage);
  socket.send(buffer);
  // socket.send(JSON.stringify(myMessage));
  // message('<ons-list-item class="action">Sent msgpack: ' + buffer.length + ' bytes.');
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

