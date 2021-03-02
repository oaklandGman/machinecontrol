$(document).ready(function() {
      var hostName = location.hostname;
      var socket;
      /*The user has WebSockets!!! */

      connect();

      function connect(){
          var host = "ws://"+hostName+"/ws";

          try{
              socket = new WebSocket(host); 

              $("#footerText").html("Connecting...");
              message('<ons-list-item class="event">Socket Status: '+socket.readyState);

              socket.onopen = function(){
             	 message('<ons-list-item class="event">Socket Status: '+socket.readyState+' (open)');
               $("#footerText").html("Connection Open");
              }

              socket.onmessage = function(msg){
               var dataMsg = msg.data + '';

               message('<ons-list-item class="message">Recv: '+dataMsg); 
              
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
        var myName = $( this ).attr('name');
        var myValue = $( this ).val();
        var myMsg = myName +"="+ myValue;
        socket.send(myMsg);
        message('<ons-list-item class="action">Sent: '+myMsg);
      });
  
      $('.button').click(function(){
        var myName = $( this ).attr('name');
        var myMessage;

        switch(myName) {
          case "btnteston":
            myMessage = {"f":"motor","c":"test","v":1};
          case "btntestoff":
            myMessage = {"f":"motor","c":"test","v":0};
  
        } // end switch

        if (myMessage){ // function, command, value
          var buffer = msgpack.encode(myMessage);
          socket.send(buffer);
          // socket.send(JSON.stringify(myMessage));
          message('<ons-list-item class="action">Sent message');
        }
      });
});

function message(msg){
  $('#diagList').prepend(msg+'</ons-list-item>');
  // ons.compile($('#diagList'));
}

