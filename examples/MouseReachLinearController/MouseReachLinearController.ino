#include <MouseReachLinearController.h>


MouseReachLinearController dev;

void setup()
{
  dev.setup();
  dev.startServer();
}

void loop()
{
  dev.update();
}
