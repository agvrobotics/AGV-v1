#include <Adafruit_VC0706.h>
#define cameraconnection Serial1

Adafruit_VC0706 camera = Adafruit_VC0706(&cameraconnection);

bool pictureTaken = false; 

void setup() {
    Serial.begin(9600);     

    if (!camera.begin()) {
        Serial.println("Camera not found!");
        return;
    }    
    camera.setImageSize(VC0706_640x480);
    Capture();
}

void loop(){
}

void Capture() {
  while (!pictureTaken) {
    if (camera.takePicture()) {
        pyserial();
        pictureTaken = true;  
    } else {
        Serial.println("Failed to take picture.");
    }
  }
}  

void pyserial() {
    uint32_t imgSize = camera.frameLength();
    Serial.println("-----------------");
    Serial.print("Image size: ");
    Serial.println(imgSize);
    Serial.println("-----------------");
    //return;
    while (imgSize > 0) {
      uint8_t bytesToRead = min((uint32_t)32, imgSize);
      uint8_t *buffer = camera.readPicture(bytesToRead); 
      if (!buffer) {
          Serial.println("Failed to read image data!");
          while (1);
      }
      Serial.write(buffer, bytesToRead);
      imgSize -= bytesToRead;
    }
}
