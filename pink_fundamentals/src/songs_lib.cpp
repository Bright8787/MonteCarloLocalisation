
#include <cstdlib>
#include <ros/ros.h>
#include "package/songs_lib.hpp"
#include "create_fundamentals/StoreSong.h"
#include "create_fundamentals/PlaySong.h"

uint8_t MEASURE = 80;
uint8_t Q = MEASURE / 4;
uint8_t Ed = MEASURE * 3 / 16;
uint8_t S = MEASURE / 16;

uint8_t r = 30;
uint8_t e3 = 52;
uint8_t a3 = 57;
uint8_t c4 = 60;
uint8_t des4 = 61;
uint8_t cis4 = des4;
uint8_t d4 = 62;
uint8_t ees4 = 63;
uint8_t dis4 = ees4;
uint8_t e4 = 64;
uint8_t f4 = 65;
uint8_t ges4 = 66;
uint8_t fis4 = ges4;
uint8_t g4 = 67;
uint8_t aes4 = 68;
uint8_t gis4 = aes4;
uint8_t a4 = 69;
uint8_t bes4 = 70;
uint8_t ais4 = bes4;
uint8_t b4 = 71;
uint8_t c5 = 72;
uint8_t des5 = 73;
uint8_t cis5 = des5;
uint8_t d5 = 74;
uint8_t e5 = 76;
uint8_t f5 = 77;
uint8_t ges5 = 78;
uint8_t fis5 = ges5;
uint8_t g5 = 79;
uint8_t aes5 = 80;
uint8_t gis5 = aes5;
uint8_t a5 = 81;
uint8_t bes5 = 82;
uint8_t ais5 = bes5;
uint8_t b5 = 83;
uint8_t c6 = 84;
uint8_t des6 = 85;
uint8_t cis6 = des6;
uint8_t d6 = 86;
uint8_t ees6 = 87;
uint8_t dis6 = ees6;
uint8_t e6 = 88;
uint8_t f6 = 89;
uint8_t ges6 = 90;
uint8_t fis6 = ges6;
// ros::ServiceClient storeSongClient;
// ros::ServiceClient playSongClient;
// storeSongClient = nh.serviceClient<create_fundamentals::StoreSong>("store_song");
// playSongClient = nh.serviceClient<create_fundamentals::PlaySong>("play_song");

void uploadSongs(ros::ServiceClient &storeSongClient) {
    create_fundamentals::StoreSong store;
    create_fundamentals::StoreSong store1;
    create_fundamentals::StoreSong store2;
    create_fundamentals::StoreSong store3;
    create_fundamentals::StoreSong store4;
    create_fundamentals::StoreSong store5;


    // store.request.number = 1;
    // store.request.song = {
    //     69, Q, 69, Q, 69, Q, 65, Ed, 72, S, // a4, a4, a4, f4, c5
    //     69, Q, 65, Ed, 72, S, 69, MEASURE / 2
    // };
    store1.request.number = 1;
    store1.request.song = { // Lost
        a3, S, d4, S // E3 – "Lo", A3 – "ooo", D4 – "st" e3, Q, a3, Ed,
       // Q, Ed, S
    };
    store2.request.number = 2; // juhu
    store2.request.song = {
        //g4, Q, b4, Q, a4, Ed, e5, S   // G4 – "Ju", B4 – "uu", A4 – "hu", E5 – "uu"
        d4, S, g4, Q, g4, S, g4, S, f4, Q // hehe
    };
    store3.request.number = 3;
    store3.request.song = { // Lost
        a3, Q, a3, Q, a3, Q, d4, Q // E3 – "Lo", A3 – "ooo", D4 – "st"
       // Q, Ed, S
    };

    store4.request.number = 4;
    store4.request.song = {
        d4, Q, e4, Q, g4, Q, e4, Q, b4, Ed, b4, Q, a4, Q,   // Never gonna give you up
        d4, Q, e4, Q, g4, Q, e4, Q, a4, Ed, a4, Q, g4, Q    // Never gonna let you down
    };

    store5.request.number = 5;
    store5.request.song = {
        d4, Q,  e4, Q,  g4, Ed, g4, Ed, g4, Ed, g4, S, g4, S, g4, S,g4,    // Baby shark
        d4, Ed, e4, Ed, g4, S,  g4, S,  g4, S,  g4, S, g4, S, g4, S,g4,  
        d4, Q,  e4, Q,  g4, Ed, g4, Ed, g4, Ed, g4, S, g4, S, g4, S,g4,    // Baby shark
        d4, Ed, e4, Ed, g4, S,  g4, S,  g4, S,  g4, S, g4, S, g4, S,g4 
    };

    if (storeSongClient.call(store5)) {
        ROS_INFO("Baby Shark uploaded as Song 5.");
    } else {
        ROS_WARN("Failed to upload Baby Shark.");
    }
    if (storeSongClient.call(store4)) {
        ROS_INFO("Rickroll uploaded as Song 4.");
    } else {
        ROS_WARN("Failed to upload Rickroll.");
    }

    if (storeSongClient.call(store1)) {
        ROS_INFO("Song 1 uploaded.");
    } else {
        ROS_WARN("Failed to upload song.");
    }
    if (storeSongClient.call(store2)) {
        ROS_INFO("Song 2 uploaded.");
    } else {
        ROS_WARN("Failed to upload song.");
    }
    if (storeSongClient.call(store3)) {
        ROS_INFO("Song 3 uploaded.");
    } else {
        ROS_WARN("Failed to upload song.");
    }

}
void playSong(uint8_t number, ros::ServiceClient &playSongClient) {
    create_fundamentals::PlaySong play;
    play.request.number = number;
    if (playSongClient.call(play) && play.response.success) {
        ROS_INFO("Song %d played successfully.", number);
    } else {
        ROS_WARN("Could not play song %d.", number);
    }
}