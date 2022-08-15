/**
*  @filename   :   Waveshare_SIM7600.cpp
*  @brief      :   Sim7600 library
*  @author     :   Anh Trung
*
*  Copyright (C) Waveshare     April 27 2018
*  http://www.waveshare.com  http://www.waveshare.net
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

#include "main.h"
#include "stm32f1xx_hal.h"
#include "Waveshare_SIM7600.h"
#include "stdio.h"
#include "string.h"
#include "uart.h"
#include <stdlib.h>
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;
extern TIM_HandleTypeDef htim2;
extern uint8_t time_data[7], Index_SIM, rx2_data, rx2_done, read_ms;
extern char Pos_Lat[15], Pos_Log[15], Sim_Data[668];
/**************************Power on Sim7x00**************************/
//char Sim_Data[500] = {0};

void SIM_PowerOff(void)
{
	HAL_GPIO_WritePin(CTR_PW4G_GPIO_Port, CTR_PW4G_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(CTR_GND4G_GPIO_Port, CTR_GND4G_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(SIM_PWON_GPIO_Port, SIM_PWON_Pin, GPIO_PIN_RESET);
}
void SIM_PowerOn(void)
{
	printf("Init SIM7600...\n\r");
	HAL_GPIO_WritePin(CTR_PW4G_GPIO_Port, CTR_PW4G_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(CTR_GND4G_GPIO_Port, CTR_GND4G_Pin, GPIO_PIN_SET);
//	HAL_Delay(500);
//	HAL_GPIO_WritePin(SIM_PWON_GPIO_Port, SIM_PWON_Pin, GPIO_PIN_SET);
//	HAL_Delay(400);
//	HAL_GPIO_WritePin(SIM_PWON_GPIO_Port, SIM_PWON_Pin, GPIO_PIN_RESET);
//  printf("SIM7600 is OK\n\r"); 
}
/*************Network environment checking*************/


void sendATcommand(char* command)
{
	char aux_str1[333];
	uint32_t time_rx2;
	//memset(aux_str1, '\0', 333);
	memset(Sim_Data, '\0', 668);
	sprintf(aux_str1, "%s\r", command);
	UART2_putString(aux_str1);
	rx2_done = 1;
	HAL_UART_Receive_IT(&huart2, &rx2_data, 1);
	time_rx2 = HAL_GetTick(); 
	while(rx2_done)
	{
		if(HAL_GetTick() - time_rx2 > 1000) rx2_done = 0;
	}
//	if(read_ms) printf("%s", Sim_Data);//UART_putStringPC(Sim_res);
//	read_ms = 1;
}
uint8_t sendAT_res(char* command, char* res)
{
	char aux_str[200];
	memset(aux_str, '\0', 200);
	uint16_t ddt = 0, ddt1 = 0, ddt2 = 0;
	sendATcommand(command);
	for(uint16_t i=0; i<Index_SIM; i++)
	{
		if(Sim_Data[i] == '\n') {
			ddt ++;  //printf("STT %d %d byte\r\n", i, ddt) ;
			if(ddt == 1) ddt1 = i + 1;	
			if(ddt == 2) ddt2 = i - 1;
		}
	}
	for(uint16_t i=ddt1; i<ddt2; i++)  aux_str[i-ddt1] = Sim_Data[i]; 
	//for(uint16_t i=ddt2; i<200; i++)  aux_str[i] = 0;
	printf("res: %s\n\r", aux_str);
	for(uint8_t i=0; i<strlen(res); i++) {
		if(aux_str[i] != res[i])  return 0;	
	}
	return 1;
}
uint8_t SIM7x00_res(char* res, char* res1){
	uint16_t ddt = 0, ddt1 = 0, ddt2 = 0;
	//char aux_str[200];
	memset(res1, '\0', 200);
	for(uint16_t i=0; i<Index_SIM; i++){
		if(Sim_Data[i] == '\n') {
			ddt ++;  //printf("STT %d %d byte\r\n", i, ddt) ;
			if(ddt == 1) ddt1 = i + 1;	
			if(ddt == 2) ddt2 = i - 1;
		}
	}
	for(uint16_t i=ddt1; i<ddt2; i++)  res1[i-ddt1] = Sim_Data[i];
	//printf("SIM7x00_res: %s\n\r", aux_str);
	for(uint8_t i=0; i<strlen(res); i++) {
		if(res1[i] != res[i])  return 0;	
	}
	return 1;
}
void Network_check(void){
	sendATcommand("AT+CPIN?");
	sendATcommand("AT+SIMEI?");
	sendATcommand("AT+CREG?");
	sendATcommand("AT+CSQ");
	sendATcommand("AT+COPS?");
}
/**************************Phone Calls**************************/

void PhoneCall(char* PhoneNumber) {
  char aux_str[30];

//  printf("Enter the phone number:");

//  scanf("%s", PhoneNumber);

  sprintf(aux_str, "ATD%s;", PhoneNumber);
  sendATcommand(aux_str);

  // press the button for hang the call 
  //while (digitalRead(button) == 1);

  //HAL_Delay(20000);

  //printfln("AT+CHUP");            // disconnects the existing call
  printf("Call disconnected\n");
}

/**************************SMS sending and receiving message **************************/
//SMS sending short message
void Sim7x00_SendSMS(char* PhoneNumber, char* Message){
  char aux_str[30];
	memset(aux_str, '\0', 30);
  printf("Sending SMS...\n\r");
  sendATcommand("AT+CMGF=1");    // sets the SMS mode to text  
  sprintf(aux_str,"AT+CMGS=\"%s\"", PhoneNumber);//0853272541
  sendATcommand(aux_str);    // send the SMS number  
	UART2_putString(Message);
	UART2_putChar(26);
}
void Sim7x00_RecSMS(char* Message){
	memset(Message, '\0', 200);
	read_ms = 0;
	sendATcommand("AT+CMGR=1");
	//HAL_Delay(200);
	//printf("DATA: %s\n\r", Sim_Data);
	if(strlen(Sim_Data) > 20){
		uint16_t ddt = 0, ddt1 = 0, ddt2 = 0;
		for(uint16_t i=0; i<Index_SIM; i++){
			if(Sim_Data[i] == '\n') {
				ddt ++;  //printf("STT %d %d byte\r\n", i, ddt) ;
				if(ddt == 2) ddt1 = i + 1;
				if(ddt == 3) ddt2 = i - 1;
			}
		}
		for(uint16_t i=ddt1; i<ddt2; i++)  Message[i-ddt1] = Sim_Data[i];		
		printf("SMS: %s\n\r", Message);
	}
	read_ms = 0;
	sendATcommand("AT+CMGD=1");
}
void PDPSetting(void){
	sendATcommand("AT+CSOCKSETPN=1");         //PDP profile number setting value:1~16
	/*********************Enable PDP context******************/
	sendATcommand("AT+CIPMODE=0");          //command mode,default:0
	sendATcommand("AT+NETOPEN");         //Open network
	//sendATcommand("AT+IPADDR");       //Return IP address
}
void TCPClient(char* ServerIP,char* Port)  //TCP Client Mode
{
	char aux_str[100];
	memset(aux_str, '\0', 100);
	sendATcommand("AT+CIPSHUT");
	HAL_Delay(500);
	//sendATcommand("AT+NETOPEN", "+NETOPEN: 0", 1000);         //Open network
	/*********************TCP client in command mode******************/
	snprintf(aux_str, sizeof(aux_str), "AT+CIPOPEN=1,\"%s\",\"%s\",%s", "TCP", ServerIP, Port);
	sendATcommand(aux_str);     //Setting tcp mode��server ip and port
	// if(sendAT_res(aux_str, "OK"))
	// {
	// 	printf("TCP OK...\r\n");
	// }
	//sendATcommand("AT+CIPSEND=1,11", ">", 2000);          //If not sure the message number,write the command like this: AT+CIPSEND=0
	//sendATcommand(Message, "OK", 2000); 
}
void TCPClientSendMS(char* Message) //TCP Client Send Message
{
	char aux_str[200];
	memset(aux_str, '\0', 200);
	sprintf(aux_str, "AT+CIPSEND=1,%d", 16);
	//sendATcommand("AT+CIPSEND?", "OK", 2000); 
	sendATcommand(aux_str);    
	HAL_Delay(200);      //If not sure the message number,write the command like this: AT+CIPSEND=0
	if(Sim_Data[Index_SIM-1] == '>')
	{ 
		if(sendAT_res(Message, "OK"))
		{
			memset(Sim_Data, '\0', 668);
			uint32_t time_tcp = HAL_GetTick();
			while(1)
			{
				if(strlen(Sim_Data) > 10)
				{
					HAL_Delay(50);
					//printf("Data SIM: %s: %c %c\r\n", Sim_Data, Sim_Data[3],Sim_Data[4]);
					printf("TCPClientSendMS OK...\r\n");
					break;
				}
				if(HAL_GetTick()-time_tcp > 3000) { printf("TCPClientSendMS fail...\r\n"); break;}
			}
		}
	}
}
void RevMS_TCP(char* Message){
	uint16_t iddt = 0, iddt1 = 0, iddt2 = 0;
	read_ms = 0;
	memset(Message, '\0', 200);
	sendATcommand("AT+CIPRXGET=2,1,200");
	if((Sim_Data[26]=='R')&&(Sim_Data[27]=='X')){
		for(uint16_t i=0; i<Index_SIM; i++){
			if(Sim_Data[i] == '\n') {
				iddt ++; //printf("STT %d %d byte\r\n", i, iddt) ;
				if(iddt == 2) iddt1 = i + 1;	
				if(iddt == 3) iddt2 = i - 1;
			}
		}
		for(uint16_t i=iddt1; i<iddt2; i++){
			Message[i-iddt1] = Sim_Data[i]; 
		}
		printf("Data MS: %s\r\n",Message);
	}
}
/**************************FTP download file to Module EFS , uploading EFS file to FTP**************************/
void ConfigureFTP(char* FTPServer, char* FTPUserName, char* FTPPassWord){
  char aux_str[100]; 
	memset(aux_str, '\0', 100);
    // sets the paremeters for the FTP server
  sendATcommand("AT+CFTPPORT=21");
  sendATcommand("AT+CFTPMODE=1");
  sendATcommand("AT+CFTPTYPE=A");
  sprintf(aux_str,"AT+CFTPSERV=\"%s\"", FTPServer);
  sendATcommand(aux_str);
  HAL_Delay(50);  
  sprintf(aux_str, "AT+CFTPUN=\"%s\"", FTPUserName);
  sendATcommand(aux_str); HAL_Delay(500);
  sprintf(aux_str, "AT+CFTPPW=\"%s\"", FTPPassWord);
  sendATcommand(aux_str); HAL_Delay(500);
} 
void UploadToFTP(char* Dir, char* FileName, char* Data){
  char aux_str[120]; 
	memset(aux_str, '\0', 120);
  printf("Upload file to FTP...\n");
  sprintf(aux_str, "AT+CFTPPUT=\"%s%s\",20", Dir, FileName);
  sendATcommand(aux_str);
	//memset(Sim_Data, '\0', 668);
	//uint32_t time_upftp = HAL_GetTick();
	HAL_Delay(5000);
	sendATcommand(Data);
	UART2_putChar(26);

}
uint8_t ConfigureFTPS(char* FTPServer, char* FTPPort, char* FTPUserName, char* FTPPassWord, char* Res){
  char aux_str[100]; 
	memset(aux_str, '\0', 100);
    // sets the paremeters for the FTP server
  sendATcommand("AT+CFTPSSTART");
  sprintf(aux_str,"AT+CFTPSLOGIN=\"%s\",%s,\"%s\",\"%s\",0", FTPServer, FTPPort, FTPUserName, FTPPassWord);
	return sendAT_res(aux_str, Res);
}

void UploadToFTPS(char* Dir, char* FileName, char* Data){
  char aux_str[120]; 
	memset(aux_str, '\0', 120);
  printf("Upload file to FTP...\n");
  sprintf(aux_str, "AT+CFTPSPUT=\"%s%s\",%d", Dir, FileName, strlen(Data));
  sendATcommand(aux_str);
	//memset(Sim_Data, '\0', 668);
	uint32_t time_upftp = HAL_GetTick();
	HAL_Delay(10);
	//printf("Data SIM: %c %c\r\n", Sim_Data[Index_SIM-2],Sim_Data[Index_SIM-1]);
	while(1){//((strlen(Sim_Data)) < 5) {
		for(uint16_t i=0; i<Index_SIM; i++) 
			if(Sim_Data[i] == '>') { sendATcommand(Data); break; }
		if(HAL_GetTick()-time_upftp > 4000) break;
	}
	//sendATcommand(Data);
}

uint8_t DownloadFromFTPS(char* FileName, char* Res){
  char aux_str[100]; 
	memset(aux_str, '\0', 100);
  printf("Download file from FTP...\n");
	//sendATcommand("AT+CFTPGETFILE=?"); \"/boot_els.bin\",0"
  sprintf(aux_str, "AT+CFTPSGETFILE=\"%s\",0", FileName);
	return sendAT_res(aux_str, Res);
}

/**************************GPS positoning**************************/
bool LBS_Positioning(void){		
  //char LatDD[3],LatMM[10],LogDD[4],LogMM[10],DdMmYy[7] ,UTCTime[7];
  char DMY_TIME[3][2], UTC_TIME[3][2];
  printf("Start LBS_Positioning...\n");
	//sendATcommand("AT+CNETSTART");
	if(sendAT_res("AT+CNETSTART", "ERROR")) {
		sendATcommand("AT+CNETSTOP"); 
		return false;
	}
	else sendATcommand("AT+CLBS=4");
	uint32_t time_LBS = HAL_GetTick();
	while(1) {
		if(strlen(Sim_Data) > 40) break;
		if(HAL_GetTick()-time_LBS > 2999) break;
	}
	if(strlen(Sim_Data)>40){
		strncpy(Pos_Log, Sim_Data+21, 10);
		strncpy(Pos_Lat, Sim_Data+11, 9);
		printf("Toa do: %s: %s\n", Pos_Lat, Pos_Log);
		for(uint8_t i=0; i<3; i++){
			strncpy(DMY_TIME[i],Sim_Data+38+3*i,2);
			time_data[6-i] = atoi(DMY_TIME[i]);
		}
		printf("Day Month Year is: %d: %d: %d\n\r", time_data[4], time_data[5], time_data[6]); 
		for(uint8_t i=0; i<3; i++){
			strncpy(UTC_TIME[i],Sim_Data+47+3*i,2);
			time_data[2-i] = atoi(UTC_TIME[i]);
		}
		printf("UTC time is: %d: %d: %d\n\r", time_data[2], time_data[1], time_data[0]);
    sendATcommand("AT+CNETSTOP");
		return true;	
	}
	sendATcommand("AT+CNETSTOP");
	return false;
}
bool GPSPositioning(void){		
  char LatDD[3],LatMM[10],LogDD[4],LogMM[10];//DdMmYy[7] ,UTCTime[7];
  char DMY_TIME[3][2], UTC_TIME[3][2];//Month[2], Year[2], Hour[2], Minute[2], Second[2];
  float Lat = 0, Log = 0;
  printf("Start GPS session...\n");
	//sendATcommand("AT+CGPS?");
  sendATcommand("AT+CGPS=1,1");    // start GPS session, standalone mode
	//sendATcommand("AT+CGPSINFO=?"); 
	sendATcommand("AT+CGPSINFO"); 
	if(strlen(Sim_Data)>70){
    strncpy(LatDD,Sim_Data+25,2);
    LatDD[2] = '\0';
    strncpy(LatMM,Sim_Data+27,9);
    LatMM[9] = '\0';   
    Lat = atoi(LatDD) + (atof(LatMM)/60);
    if(Sim_Data[37] == 'N') sprintf(Pos_Lat, "%fN", Lat);
    else {
			if(Sim_Data[37] == 'S') sprintf(Pos_Lat, "%fS", Lat);
    }
    strncpy(LogDD,Sim_Data+39,3);
    LogDD[3] = '\0';
    strncpy(LogMM,Sim_Data+42,9);
    LogMM[9] = '\0';
    Log = atoi(LogDD) + (atof(LogMM)/60);
    if(Sim_Data[52] == 'E') sprintf(Pos_Log, "%fE", Log);
    else {
			if(Sim_Data[52] == 'W') sprintf(Pos_Log, "%fW", Log); 
    }
		for(uint8_t i=0; i<3; i++){
			strncpy(DMY_TIME[i],Sim_Data+54+2*i,2);
			time_data[i+4] = atoi(DMY_TIME[i]);
		}
		printf("Day Month Year is: %d: %d: %d\n\r", time_data[4], time_data[5], time_data[6]); 
    for(uint8_t i=0; i<3; i++){
			strncpy(UTC_TIME[i],Sim_Data+61+2*i,2);
			time_data[2-i] = atoi(UTC_TIME[i]);
		}
		printf("UTC time is: %d: %d: %d\n\r", time_data[2], time_data[1], time_data[0]);
		memset(Sim_Data, '\0', 668);
		return true;
	}
	return false;
}

/**************************Other functions**************************/




