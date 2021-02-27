$(document).ready(function() {
      var hostName = location.hostname;
      /*The user has WebSockets!!! */

      connect();

      function connect(){
          var socket;
          var host = "ws://"+hostName+":80";

          try{
              var socket = new WebSocket(host); 

              $("#footerText").html("Connecting...");
              message('<li class="event">Socket Status: '+socket.readyState);

              socket.onopen = function(){
             	 message('<li class="event">Socket Status: '+socket.readyState+' (open)');
               $("#footerText").html("Connected!");
              }

              socket.onmessage = function(msg){
               var dataMsg = msg.data + '';

               message('<li class="message">Msg: '+dataMsg); 
              
              }
              

              socket.onclose = function(){
              	message('<li class="event">Socket Status: '+socket.readyState+' (Closed)');
                 $("#footerText").html("Connection lost!");
                 setTimeout(connect(), 2000);
              }			

          } catch(exception){
             message('<li>Error'+exception);
          }

          function message(msg){
            $('#diagList').prepend(msg+'</li>');
            $('#diagList').listview("refresh");
          }

       
          $(".numbox").on("slidestop", function(){
            var myName = $( this ).attr('name');
            var myValue = $( this ).val();
            var myMsg = myName +"="+ myValue;
            socket.send(myMsg);
            message('<li class="message">Sent: '+myMsg);
          });

          $(":button").click(function(){
            var myName = $( this ).attr('name');
            var myValue = $( this ).html();
            var myAction = myName +"="+ myValue;
            if (myValue=="Start") { /* user clicked start, make sure controller has all the data first */
              $(".numbox").each(function(){
                var myName = $( this ).attr('name');
                var myValue = $( this ).val();
                var myMsg = myName +"="+ myValue;
                socket.send(myMsg);
                message('<li message="event">Sent: '+myMsg);
              });
            }
            socket.send(myAction);
            message('<li class="message">Sent: '+myAction);
            event.preventDefault();
          });

      }//End connect

});