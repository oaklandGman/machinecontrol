$(document).ready(function() {
      var hostName = location.hostname;
      /*The user has WebSockets!!! */

      connect();

      function connect(){
          var host = "ws://"+hostName+"/ws";

          try{
              var socket = new WebSocket(host); 

              $("#footerText").html("Connecting...");
              message('<ons-list-item class="event">Socket Status: '+socket.readyState);

              socket.onopen = function(){
             	 message('<ons-list-item class="event">Socket Status: '+socket.readyState+' (open)');
               $("#footerText").html("Connected!");
              }

              socket.onmessage = function(msg){
               var dataMsg = msg.data + '';

               message('<ons-list-item class="message">Msg: '+dataMsg); 
              
              }
              

              socket.onclose = function(){
              	message('<ons-list-item class="event">Socket Status: '+socket.readyState+' (Closed)');
                 $("#footerText").html("Connection lost!");
                 setTimeout(connect(), 2000);
              }			

          } catch(exception){
             message('<ons-list-item class="error">Error '+exception);
          }
      }//End connect



});

function message(msg){
  $('#diagList').prepend(msg+'</ons-list-item>');
  // ons.compile($('#diagList'));
}