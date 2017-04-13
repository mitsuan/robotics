#include <EEPROM.h>
#include <Ethernet.h>
#include<SPI.h>

//device configuration
byte mac[] = {0x00, 0xAA, 0xBB, 0xCC, 0xDE, 0x02};
EthernetServer server = EthernetServer(12358);
IPAddress ip(192, 168, 0, 102);
//IPAddress ip(192, 168,100,200);

//Actuator for lock
byte lock=0;


int endGet=0;

void setup() {
  
  Serial.begin(9600);
  boolean receiving = false;
  
  Ethernet.begin(mac,ip); 
  Serial.print("LocalIP address of the arduino:");
  Serial.println(Ethernet.localIP());

  
  server.begin();
  Serial.println("Server Started");
  
}

void loop() {
  
  Serial.println("Waiting for a client to connect");
  EthernetClient client = server.available();

if(client) {
  

  Serial.print("Connected to a client : ");

  String getReq="";

  while(client.available()) {             
    
      char c = client.read();
       Serial.print(c);


        if(endGet==0)
        {
          if(c=='H')
         {
          endGet=1;

          Serial.print("GET request: ");
          Serial.println(getReq.substring(5));
         }
         else
         {
           getReq+=(char)c;

         }
         
        }
       
      String uid=getReq.substring(getReq.indexOf("uid")+4,getReq.indexOf("&"));
      String pass=getReq.substring(getReq.indexOf("pass")+5,getReq.lastIndexOf(" "));
    
      if(search(uid,pass))
      {
              
      }
    else
    {
    }
           
       
      
  }
  
  if(client.connected()) {
      Serial.println("Response Sent to Client: A HTML Page");
      client.println("HTTP/1.1 200 OK");
      client.println("Content-Type: text/html\n");
      client.print("<center><h2>Robotics society welcomes you!!!</h2></center>");
  }  
  
delay(5);
client.stop();
Serial.println("Client is disconnected");
}

delay(2000); // Wait for 2 seconds before starting to listen again
}

void search(String uid,String pass)
{
  for(int i=0;i<EEPROM.length();i+=15)
  {
    
    
  }
  
}


