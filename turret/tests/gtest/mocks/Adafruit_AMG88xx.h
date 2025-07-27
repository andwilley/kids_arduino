#ifndef ADAFRUIT_AMG88XX_H
#define ADAFRUIT_AMG88XX_H

#define AMG88xx_PIXEL_ARRAY_SIZE 64

class Adafruit_AMG88xx {
public:
  bool begin(int = 0) { return true; }
  void readPixels(float *) {}
};

#endif // ADAFRUIT_AMG88XX_H
