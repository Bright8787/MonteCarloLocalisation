
#ifndef SONGS_LIB
#define SONGS_LIB

#pragma once
#include <vector>
#include "create_fundamentals/StoreSong.h"
#include "create_fundamentals/PlaySong.h"

extern uint8_t MEASURE;
extern uint8_t Q;
extern uint8_t Ed;
extern uint8_t S;

// Note values
extern uint8_t r, e3, a3, c4, des4, cis4, d4, ees4, dis4, e4, f4;
extern uint8_t ges4, fis4, g4, aes4, gis4, a4, bes4, ais4, b4, c5;
extern uint8_t des5, cis5, d5, e5, f5, ges5, fis5, g5, aes5, gis5;
extern uint8_t a5, bes5, ais5, b5, c6, des6, cis6, d6, ees6, dis6;
extern uint8_t e6, f6, ges6, fis6;
extern ros::ServiceClient storeSongClient;
void uploadSongs(ros::ServiceClient &storeSongClient);
void playSong(uint8_t number, ros::ServiceClient &playSongClient);
#endif // SONGS_H
