/**
*  @filename   :   Waveshare_SIM7600.h
*  @brief      :   Sim7600 library
*  @author     :   Anh Trung
*
*  Copyright (C) Waveshare     April 27 2018
*  http://www.waveshare.com / http://www.waveshare.net
*
* Permission is hereby granted, free of charge, to any person obtaining a copy
* of this software and associated documnetation files (the "Software"), to deal
* in the Software without restriction, including without limitation the rights
* to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
* copies of the Software, and to permit persons to  whom the Software is
* furished to do so, subject to the following conditions:
*
* The above copyright notice and this permission notice shall be included in
* all copies or substantial portions of the Software.
*
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
* FITNESS OR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
* AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
* LIABILITY WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
* OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
* THE SOFTWARE.
*/

#include "stdlib.h"
#include "stdio.h"
#include "string.h"
#include "stdbool.h"
void deleteBuffer(char* buf);
void sendATcommand(char* command);
uint8_t sendAT_res(char* command, char* res);
uint8_t SIM7x00_res(char* res, char* res1);
// SIM query
void SIM_PowerOn(void);
void SIM_PowerOff(void);
void Network_check(void);
// Phone calls
void PhoneCall(char* PhoneNumber);

// SMS sending and receiving message 
void Sim7x00_SendSMS(char* PhoneNumber, char* Message);
void Sim7x00_RecSMS(char* Message);

// FTP download file to Module EFS or uploading EFS file to FTP
void ConfigureFTP(char* FTPServer, char* FTPUserName, char* FTPPassWord);
void UploadToFTP(char* Dir, char* FileName, char* Data);
uint8_t ConfigureFTPS(char* FTPServer, char* FTPPort, char* FTPUserName, char* FTPPassWord, char* Res);
void UploadToFTPS(char* Dir, char* FileName, char* Data);
uint8_t DownloadFromFTPS(char* FileName, char* Res);
//void Putfile(char* FileName);
// GPS positoning
bool GPSPositioning(void);
bool LBS_Positioning(void);
//TCP and UDP communication
void PDPSetting(void);
void TCPClient(char* ServerIP,char* Port);  //TCP Client Mode
void TCPClientSendMS(char* Message);  //TCP Client Send Message
void RevMS_TCP(char* Message);
//  bool UDPServerCM(char* ServerIP,char* Port,char* Message,char* MessageSize);  //UDP Client Command Mode
//  bool TCPServerCM(char* ServerIP,char* Port,char* Message,char* MessageSize);  //TCP Client Command Mode

// Other functions.
//void display(char* data);

//char sendATcommand2(char* ATcommand, char* expected_answer1, char* expected_answer2, unsigned int timeout);




