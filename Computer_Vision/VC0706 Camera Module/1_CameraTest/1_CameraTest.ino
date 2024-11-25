#include <Adafruit_VC0706.h>
#define cameraconnection Serial1

Adafruit_VC0706 camera = Adafruit_VC0706(&cameraconnection);

bool pictureTaken = false; 

void setup() {
    Serial.begin(9600);     
    
    if (!camera.begin()) {
      Serial.println("Camera not found. Check wiring!");
    }
    Camera();
}

void loop(){
}

void Camera(){
  char *reply = camera.getVersion();
  Serial.println("-----------------");
  if (reply == 0) {
    Serial.print("Failed to get version");
  } else {
    Serial.print(reply);
  }
  Serial.println("-----------------");
}
