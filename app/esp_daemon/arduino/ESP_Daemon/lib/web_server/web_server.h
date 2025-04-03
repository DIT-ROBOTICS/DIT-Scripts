#ifndef WEB_SERVER_H
#define WEB_SERVER_H

#include <ESPAsyncWebServer.h>

extern AsyncWebServer server;
extern AsyncEventSource events;

void initSPIFFS();
void initWebServer();

#endif
