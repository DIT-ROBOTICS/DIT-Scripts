#ifndef ESPNOW_COMM_H
#define ESPNOW_COMM_H

typedef struct {
    int sima_start;
} struct_message;

void initESPNow();
void sendESPNow(int data);

#endif
