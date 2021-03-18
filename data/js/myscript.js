ons.ready(function() {
  var hostName = location.hostname;
  var socket;
  /*The user has WebSockets!!! */

  try{
    let acl = new Accelerometer({frequency: 60});
    acl.addEventListener('reading', () => {
      document.getElementById("x_axis").value = acl.x;
      document.getElementById("y_axis").value = acl.y;
      document.getElementById("z_axis").value = acl.z;
      // console.log("Acceleration along the X-axis " + acl.x);
      // console.log("Acceleration along the Y-axis " + acl.y);
      // console.log("Acceleration along the Z-axis " + acl.z);
    });

    acl.start();
  }
  catch(e){
    alert("Accelerometer error " + e);
  }
  

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
                var myKey = dataMsg.substring(0,4);
                if(myKey=="Exec" || myKey=="Prog") { // received program exec message
                  if (dataMsg.indexOf("file")>0) { // make filenames tappable
                    progInfo('<ons-list-item tappable modifier="longdivider" class="programfile">'+dataMsg);
                  } else { // stuff that's not a filename
                    progInfo('<ons-list-item modifier="longdivider" class="progrmsg">'+dataMsg);
                  }
                } else if(myKey=="Conf") { // received program exec message
                  configList('<ons-list-item class="config">'+dataMsg);
                } else { // other message
                  message('<ons-list-item class="message">'+dataMsg);
                }
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

    // program list
    $(document).on('click','#proginfo div', function(e){
      var myText = $( this ).text();
      // alert(myText);
      if (myText.indexOf("file")>0) { // only handle program file names
        var myProgname = myText.substring(myText.indexOf("/"));
        document.getElementById("progname").value = myProgname;
      }
    });

      // BUTTONS
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
      } else if (myId=="loadprog") {
        var myFilename = document.getElementById("progname").value;
        if (myFilename.length > 1 && myFilename.substring(0,1)=="/") {
          myMessage.txt = myFilename; // assign filename
          myMessage.dat = 0x01;
          progInfo("CLEARLIST"); // clear program info list
        } else {
          alert("Problem with file name, maybe missing leading /");
          return;
        }
      } 
  
      if (myMessage.dat != null){ // function, command, value
        sendPack(myMessage, socket);
      }
      event.preventDefault();
    }); // end button click

    // SWITCHES
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
      } else if (myId=="dualspeed") {
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

    // NUMBER BOXES
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
  if (msg == "CLEARLIST") $('#diagList').empty();
  else $('#diagList').prepend(msg+'</ons-list-item>');
}

function progInfo(msg){
  if (msg == "CLEARLIST") $('#proginfo').empty();
  else $('#proginfo').prepend(msg+'</ons-list-item>');
}

function configList(msg){
  if (msg == "CLEARLIST") $('#configlist').empty();
  else $('#configlist').prepend(msg+'</ons-list-item>');
}

function isJson(str) {
  try {
    JSON.parse(str);
  } catch (e) {
    return false;
  }
  return true;
}

