/*
 * To change this license header, choose License Headers in Project Properties.
 * To change this template file, choose Tools | Templates
 * and open the template in the editor.
 */

/* 
 * File:   main.cpp
 * Author: Vivek,john solomon
 *
 * Created on 17 April, 2018, 12:54 PM
 */

#include <cstdlib>
#include "MFRC522.h"
#include <zmq.hpp>
#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <bcm2835.h>
#include <sstream>
#include <fstream>
#include <string>
#include <iostream>
#include <cstdio>
#include <ctime>
#include <termios.h>
#include <cstring>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <termios.h>
#include <sys/ioctl.h>
#include "rapidjson/document.h"     // rapidjson's DOM-style API
#include "rapidjson/prettywriter.h" // for stringify JSON
#include "utills/system_support.h"
#define capsense_pin 17
using namespace std;
using namespace rapidjson;

MFRC522 mfrc; //rfid object to access structure
/*******************************************************************************************************/
//varibles declaration
bool sense_val[8]; //Qre sensor read
char buf1, buf2[100]; //ARRAY in which data will be send and receive
speed_t myBaud; //Used to configure baud rate
int fd; //file descriptor
pthread_t thread_1; // thread id 
string key;
Document document;
//function declration
/***************************************************************************************************************/
bool speech_flag = 0; //stop 
void setup();
struct termios config_serial;
void* mode_select(void *arg); //mode selection
void line_follower(void); //line follower algo
void rf_read(void); //read rfid card
void play(void);
void teach(void);
void learn(void);
void send_data(char);
void recv_data();
int baudrate_set(const unsigned int);
void configure_serial(void);

/******************************************************************************************************************/
int main(int argc, char** argv) 
{   setup();
    //rf_read();
    zmq::context_t context(1);
    zmq::socket_t socket(context, ZMQ_REP);
    socket.bind("tcp://*:5555");
    StringBuffer s;
    Writer<StringBuffer> writer(s);
    writer.StartObject(); // Between StartObject()/EndObject(), 
    writer.Key("status");
    writer.String("started");
    writer.Key("boardId");
    writer.String("1234567890");
    writer.EndObject();
    cout << "MOTORCORTEX::" << s.GetString() << endl;



    while (1) {
        //setup();
        std::string data;
        zmq::message_t request;
        //  Wait for next request from client
        socket.recv(&request);
        data = std::string(static_cast<char*> (request.data()), request.size());
        zmq::message_t reply(8);
        memcpy(reply.data(), "Received", 8);
        socket.send(reply);
        if (!SystemSupport::starts_with(data, " ")) {
            data = " " + data + " ";
        }
            if (document.Parse<0>(data.c_str()).HasParseError()) {
        cout << "MOTORCORTEX::{\"status\" : \"error in parse\"}" << endl;
       continue;
    }
   

        //cout<<"LOG::"<<key<<endl;
        
        cout<<"LOG::"<<data<<endl;
        thread_1 = pthread_create(&thread_1, NULL, &mode_select, NULL);
        pthread_cancel(thread_1);
        



        // TODO : do the check and do the needful
        //  mode_select(key); // mode or instruction
    }
    return 0;
}

void* mode_select(void *arg)
 {
   // string mode = *(string*) arg;
    string mode = document["mode"].GetString();
    cout << "thread_created" << endl;
   // send_data('T');
    cout << "mode"<<mode<< endl;
    if (mode == "play" || mode == "play mode") {
        bool exit_flag=false;
        cout<<"entered into play mode"<<endl;
        //while(exit_flag !=true)
        {
        string direction = document["instructions"].GetString();
        cout<<"direction"<<direction<<endl;
        //TODO: send the direction to controller
        if(direction == "forward")
        {
            cout<<"Moving forward"<<endl;
            send_data('F');      
        StringBuffer s;
        Writer<StringBuffer> writer(s);
        writer.StartObject(); // Between StartObject()/EndObject(), 
        writer.Key("state");
        writer.String("next");
        writer.Key("mode");
        writer.String("play");
        writer.EndObject();
        cout << "MOTORCORTEX::" << s.GetString() << endl;
        
        }
        else if(direction == "right")
        {
            cout<<"Moving right"<<endl;
            send_data('R'); 
        StringBuffer s;
        Writer<StringBuffer> writer(s);
        writer.StartObject(); // Between StartObject()/EndObject(), 
        writer.Key("state");
        writer.String("next");
        writer.Key("mode");
        writer.String("play");
        writer.EndObject();
        cout << "MOTORCORTEX::" << s.GetString() << endl;
        }
        else if(direction == "left")
        {
            cout<<"Moving left"<<endl;
            send_data('L'); 
        StringBuffer s;
        Writer<StringBuffer> writer(s);
        writer.StartObject(); // Between StartObject()/EndObject(), 
        writer.Key("state");
        writer.String("next");
        writer.Key("mode");
        writer.String("play");
        writer.EndObject();
        cout << "MOTORCORTEX::" << s.GetString() << endl;
        }
        else if(direction == "dance")
        {
            cout<<"dance"<<endl;
            send_data('D'); 
        StringBuffer s;
        Writer<StringBuffer> writer(s);
        writer.StartObject(); // Between StartObject()/EndObject(), 
        writer.Key("state");
        writer.String("next");
        writer.Key("mode");
        writer.String("play");
        writer.EndObject();
        cout << "MOTORCORTEX::" << s.GetString() << endl;
        }
        else if(direction == "back")
        {
            cout<<"dance"<<endl;
            send_data('B');
        StringBuffer s;
        Writer<StringBuffer> writer(s);
        writer.StartObject(); // Between StartObject()/EndObject(), 
        writer.Key("state");
        writer.String("next");
        writer.Key("mode");
        writer.String("play");
        writer.EndObject();
        cout << "MOTORCORTEX::" << s.GetString() << endl;
        }
        }

    } else if (mode == "code") {
                //        for (SizeType i = 0; i < a.Size(); i++)
        //        {
        //            
        //        }
        cout << "learn" << endl;
    } else if (mode == "teach") {
        cout << "teach" << endl;
    }

    pthread_exit(NULL);
}

void setup() {

    baudrate_set(9600);
    configure_serial();
    bcm2835_gpio_fsel(17, BCM2835_GPIO_FSEL_INPT);
}

void rf_read(void) {

    while (1) {

        mfrc.PCD_Init();
        MFRC522::MIFARE_Key key;
        for (byte i = 0; i < 6; i++) key.keyByte[i] = 0xFF;
        MFRC522::StatusCode status;
        if (!mfrc.PICC_IsNewCardPresent()) {
            continue;
        }
        if (!mfrc.PICC_ReadCardSerial()) {
            cout << "card detected" << endl;
            continue;
        }
        byte buffer1[18], buffer2[18], len = 18, block = 1;
        mfrc.PCD_Authenticate(MFRC522::PICC_CMD_MF_AUTH_KEY_A, 1, &key, &(mfrc.uid));
        mfrc.MIFARE_Read(block, buffer1, &len);
        for (uint8_t i = 0; i < 16; ++i) {
            // printf("%X",buffer1[i]);
            if (buffer1[i] != 0x20) {
                buffer1[i] = char(buffer1[i]);
            }
            cout << buffer1[i];
        }
        cout << "\n";
        cout << "end of reading";
        usleep(1000000);
        mfrc.PICC_HaltA();
        mfrc.PCD_StopCrypto1();
    }
}

void send_data(char buf1) {
    cout<<"wRITING"<<endl;
    write(fd, &buf1, 1);
    usleep(1000000);

}

void recv_data() 
{
    read(fd, buf2, 1);
    cout<<buf2<<endl;   
}

void configure_serial() {
      
fcntl (fd, F_SETFL, O_RDWR) ;

// Get and modify currentconfig_serial:

  tcgetattr (fd, &config_serial) ;
fd = open ("/dev/serial0", O_RDWR | O_NOCTTY | O_NDELAY | O_NONBLOCK);
    cfmakeraw   (&config_serial) ;
    cfsetispeed (&config_serial, myBaud) ;
    cfsetospeed (&config_serial, myBaud) ;

   config_serial.c_cflag |= (CLOCAL | CREAD) ;
   config_serial.c_cflag &= ~PARENB ;
   config_serial.c_cflag &= ~CSTOPB ;
   config_serial.c_cflag &= ~CSIZE ;
   config_serial.c_cflag |= CS8 ;
   config_serial.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG) ;
   config_serial.c_oflag &= ~OPOST ;

   config_serial.c_cc [VMIN]  =   0 ;
   config_serial.c_cc [VTIME] = 100 ;	// Ten seconds (100 deciseconds)

  tcsetattr (fd, TCSANOW, &config_serial) ;

}

int baudrate_set(const unsigned int baud) {
    switch (baud) {
        case 50: myBaud = B50;
            break;
        case 75: myBaud = B75;
            break;
        case 110: myBaud = B110;
            break;
        case 134: myBaud = B134;
            break;
        case 150: myBaud = B150;
            break;
        case 200: myBaud = B200;
            break;
        case 300: myBaud = B300;
            break;
        case 600: myBaud = B600;
            break;
        case 1200: myBaud = B1200;
            break;
        case 1800: myBaud = B1800;
            break;
        case 2400: myBaud = B2400;
            break;
        case 4800: myBaud = B4800;
            break;
        case 9600: myBaud = B9600;
            break;
        case 19200: myBaud = B19200;
            break;
        case 38400: myBaud = B38400;
            break;
        case 57600: myBaud = B57600;
            break;
        case 115200: myBaud = B115200;
            break;
        case 230400: myBaud = B230400;
            break;
        case 460800: myBaud = B460800;
            break;
        case 500000: myBaud = B500000;
            break;
        case 576000: myBaud = B576000;
            break;
        case 921600: myBaud = B921600;
            break;
        case 1000000: myBaud = B1000000;
            break;
        case 1152000: myBaud = B1152000;
            break;
        case 1500000: myBaud = B1500000;
            break;
        case 2000000: myBaud = B2000000;
            break;
        case 2500000: myBaud = B2500000;
            break;
        case 3000000: myBaud = B3000000;
            break;
        case 3500000: myBaud = B3500000;
            break;
        case 4000000: myBaud = B4000000;
            break;

        default:
            return -2;
    }
}
