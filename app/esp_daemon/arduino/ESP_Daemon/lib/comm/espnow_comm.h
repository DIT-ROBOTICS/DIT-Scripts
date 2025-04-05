#ifndef ESPNOW_COMM_H
#define ESPNOW_COMM_H

#include <esp_now.h>

typedef struct {
    int sima_start;
} struct_message;

void initESPNow();
void sendESPNow(int mode);

#endif
