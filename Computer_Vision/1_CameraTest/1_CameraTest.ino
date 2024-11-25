#include <Adafruit_VC0706.h>
#define cameraconnection Serial1

//Adafruit_VC0706 camera = Adafruit_VC0706(&Serial1); 
Adafruit_VC0706 camera = Adafruit_VC0706(&cameraconnection);

bool pictureTaken = false; 

void setup() {
    Serial.begin(9600);     
    //Serial1.begin(38400);

    if (camera.begin()) {
        Serial.println("Camera found!");
    } else {
        Serial.println("Camera not found. Check wiring!");
        return;
    }
    Camera();
    Capture();
}

void loop(){
}

void Camera(){
  char *reply = camera.getVersion();
  if (reply == 0) {
    Serial.print("Failed to get version");
  } else {
    Serial.println("-----------------");
    Serial.print(reply);
  }

  uint8_t imgsize = camera.getImageSize();
  Serial.print("Image resolution: ");
  if (imgsize == VC0706_640x480) Serial.println("640x480");
  if (imgsize == VC0706_320x240) Serial.println("320x240");
  if (imgsize == VC0706_160x120) Serial.println("160x120");
  Serial.println("-----------------");
}
