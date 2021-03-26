ons.ready(function() {
  var hostName = location.hostname;
  var socket;
  /*The user has WebSockets!!! */

  document.addEventListener('prechange', function(event) { // update top of page toolbar with name of tab
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
              console.log(dataMsg);
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
      if (myId=="info") { // memory info
        myMessage.fnc = "info";
        myMessage.dat = 0x01;
      } else if (myId=="loadprog") { // program loading
        var myFilename = document.getElementById("progname").value;
        if (myFilename.length > 1 && myFilename.substring(0,1)=="/") {
          myMessage.txt = myFilename; // assign filename
          myMessage.dat = 0x01;
          progInfo("CLEARLIST"); // clear program info list
        } else {
          alert("Problem with file name, maybe missing leading /");
          return;
        }
      } else if (myId=="accstartstop") { // accel stuff
        let demo_button = document.getElementById("accstartstop");

        // Request permission for iOS 13+ devices
        if (
          DeviceMotionEvent &&
          typeof DeviceMotionEvent.requestPermission === "function"
        ) {
          DeviceMotionEvent.requestPermission();
        }
        
        if (is_running){
          window.removeEventListener("devicemotion", handleMotion);
          // window.removeEventListener("deviceorientation", handleOrientation);
          demo_button.innerText = "Start Reading";
          // demo_button.classList.add('btn-success');
          // demo_button.classList.remove('btn-danger');
          is_running = false;
        }else{
          window.addEventListener("devicemotion", handleMotion);
          // window.addEventListener("deviceorientation", handleOrientation);
          document.getElementById("accstartstop").innerText = "Stop Reading";
          // demo_button.classList.remove('btn-success');
          // demo_button.classList.add('btn-danger');
          is_running = true;
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
      } else if (myId=="motsleep") {
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
      } else if (myId=="rampdepth") {
        myMessage.fnc = "motor"; 
        myMessage.cmd = myId; 
        if (myValue) { // checked
          myMessage.dat = 0x01;
        } else { // unchecked
          myMessage.dat = 0x00;
        }
      } else if (myId=="rampspeed" || myId=="rampspeedtime") {
        myMessage.fnc = "motor"; 
        myMessage.cmd = myId; 
        if (myValue) { // checked
          myMessage.dat = 0x01;
        } else { // unchecked
          myMessage.dat = 0x00;
        }
      } else if (myId=="ramplength" || myId=="ramplengthtime") {
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

function handleOrientation(event) {
  updateFieldIfNotNull('Orientation_a', event.alpha);
  updateFieldIfNotNull('Orientation_b', event.beta);
  updateFieldIfNotNull('Orientation_g', event.gamma);
  incrementEventCount();
}

function incrementEventCount(){
  let counterElement = document.getElementById("num-observed-events")
  let eventCount = parseInt(counterElement.innerHTML)
  counterElement.innerHTML = eventCount + 1;
}

function updateFieldIfNotNull(fieldName, value, precision=10){
  if (value != null)
    document.getElementById(fieldName).value = value.toFixed(precision);
}

function handleMotion(event) {
  // updateFieldIfNotNull('x_axis', event.accelerationIncludingGravity.x);
  // updateFieldIfNotNull('y_axis', event.accelerationIncludingGravity.y);
  // updateFieldIfNotNull('z_axis', event.accelerationIncludingGravity.z);

  updateFieldIfNotNull('x_axis', event.acceleration.x);
  updateFieldIfNotNull('y_axis', event.acceleration.y);
  updateFieldIfNotNull('z_axis', event.acceleration.z);

  // updateFieldIfNotNull('Accelerometer_i', event.interval, 2);

  // updateFieldIfNotNull('Gyroscope_z', event.rotationRate.alpha);
  // updateFieldIfNotNull('Gyroscope_x', event.rotationRate.beta);
  // updateFieldIfNotNull('Gyroscope_y', event.rotationRate.gamma);
  // incrementEventCount();
}

var accel_running = false;

/*
Light and proximity are not supported anymore by mainstream browsers.
window.addEventListener('devicelight', function(e) {
   document.getElementById("DeviceLight").innerHTML="AmbientLight current Value: "+e.value+" Max: "+e.max+" Min: "+e.min;
});

window.addEventListener('lightlevel', function(e) {
   document.getElementById("Lightlevel").innerHTML="Light level: "+e.value;
});

window.addEventListener('deviceproximity', function(e) {
   document.getElementById("DeviceProximity").innerHTML="DeviceProximity current Value: "+e.value+" Max: "+e.max+" Min: "+e.min;
});

window.addEventListener('userproximity', function(event) {
   document.getElementById("UserProximity").innerHTML="UserProximity: "+event.near;
});
*/