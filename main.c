/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2022 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>

#include <stdio.h>
#include <string.h>
#include <unistd.h>

#include <_ansi.h>
#include <_syslist.h>
#include <errno.h>
#include <sys/time.h>
#include <sys/times.h>
#include <limits.h>
#include <signal.h>
#include <stdint.h>
#include <sys/stat.h>
#include "w5500_spi.h"
#include "wizchip_conf.h"
#include "socket.h"
#include "w5500.h"
#include "EEPROM.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
typedef unsigned char       	UI8_t;
typedef   signed char        	SI8_t;
typedef unsigned short     		UI16_t;
typedef   signed short      	SI16_t;
typedef unsigned long      		UI32_t;
typedef   signed long       	SI32_t;
typedef unsigned long long 		UI64_t;
typedef   signed long long  	SI64_t;
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define HEADER  '$'
#define FOOTER  '#'
#define HIGH 1
#define LOW 0
#define STX 0x02
#define ETX 0x03
#define RECEIVE_BUFF_SIZE 128
#define UDP_SOCKET 2         // Additional UDP socket for discovery
#define DISCOVERY_PORT 9008 // Port for discovery service
#define BUFFER_SIZE 512      // Buffer size for data
uint8_t udp_buffer[BUFFER_SIZE]; // Buffer for UDP data


#ifndef _RETARGET_H__
#define _RETARGET_H__



void RetargetInit(UART_HandleTypeDef *huart);
int _isatty(int fd);
int _write(int fd, char* ptr, int len);
int _close(int fd);
int _lseek(int fd, int ptr, int dir);
int _read(int fd, char* ptr, int len);
int _fstat(int fd, struct stat* st);

#endif //#ifndef _RETARGET_H__
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
 ADC_HandleTypeDef hadc1;

I2C_HandleTypeDef hi2c2;

SPI_HandleTypeDef hspi1;
DMA_HandleTypeDef hdma_spi1_rx;

TIM_HandleTypeDef htim15;

UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart1_rx;
DMA_HandleTypeDef hdma_usart2_rx;

/* USER CODE BEGIN PV */
static void PHYStatusCheck(void);
static void PrintPHYConf(void);
void Tcp_Msg_send(uint8_t *);
void checkConnect(void);
void User_IP_Write(void);
void User_IP_Read(uint16_t fno);
void User_IP_Set(void);
void Default_Static_IP_Set(void);
void checkConnectDefaultIP(void);
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_USART1_UART_Init(void);
static void MX_USART2_UART_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM15_Init(void);
static void MX_SPI1_Init(void);
static void MX_I2C2_Init(void);
/* USER CODE BEGIN PFP */
volatile uint32_t g_ulTimer0TickCtr,g_ulTempSensorTickCtr,g_ulT0w5500RstCtr = 0;
uint8_t tempbuff[0],tempbuff2[0];
UI8_t g_ucRcvdUart1Char=0;
uint8_t Rx_data[256];
uint8_t Rx1Buffer[12];
uint8_t Rx1_data[256];
int8_t Rx2_data[256];
uint8_t U0_RevLocl_Buf[30];
UI8_t Device_Cmd_mode,Device_Write_mode = 0;
int count =0;
int j=0;
uint32_t g_ADCValue,g_uiTemperature;
uint8_t bln_TimerInterruptFlag1 =0;
uint8_t bln_TimerInterruptFlag2 =0;
UI8_t blnEtcLoopDetected = 0;
UI8_t blnUart0DebugMode = 0;
UI8_t blnUart1DebugMode = 0;
uint8_t blnUart1Printf = 0;
UI8_t blnFiveSecondTimeoutTIMER1,blnFiveSecondHBTIMER1,bln_ConEstablishedCkeck,bln_ConEstablishChk_ser = 0;
UI8_t bln_TimerInterruptFlag =0;
int len,ring,i=0;
char Sck_Create_success,Sending_SuccessFlag,Connected_with_server = 0;
float datar3;
uint8_t datar4[30];
uint8_t User_destination_ip[5];
uint16_t User_destination_port;

uint8_t eepromDataRecevingFlag = 0;
uint8_t eepromDataReceivedFlag = 0;

wiz_NetInfo gWIZNETINFO = {
		.mac = {0x80, 0x34, 0x28,0x74,0xA5,0xCB},
		.ip = {10,10,1,208},
		.sn = {255,0,0,0},
		.gw = {10,10,1,254},
		.dns = {10,10,1,254},
		.dhcp = NETINFO_STATIC
};

wiz_NetInfo gWIZNETINFO2;
//Additional global variable
uint8_t receive_buff[RECEIVE_BUFF_SIZE];//to receive data from client
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
#ifdef __GNUC__
#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif

PUTCHAR_PROTOTYPE
{
	setvbuf(stdout, NULL, _IONBF, 0);     // used to fflush(stdout)
	if(blnUart1Printf == 1 )
	{
		HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
	}
	else
	{
		HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
	}

	return ch;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if(huart->Instance==USART1)
	{
		HAL_UART_Receive_IT(&huart1,Rx1_data,1);
		g_ucRcvdUart1Char = Rx1_data[0];
		send(1,&g_ucRcvdUart1Char,1);
		memset(Rx1_data,'\0',256);

	}
	else
	{
		if(huart->Instance==USART2)
		{
			HAL_UART_Receive_IT(&huart2,Rx2_data,1);
			//send(1,&g_ucRcvdUart1Char,1);
			memset(Rx2_data,'\0',256);


		}
	}
}

/*wiz_NetInfo gWIZNETINFO = {
		.mac = {0x80, 0x34, 0x28,0x74,0xA5,0xCB},
		.ip = {10,10,1,202},
		.sn = {255,0,0,0},
		.gw = {10,10,1,254},
		.dns = {10,10,1,254},
		.dhcp = NETINFO_STATIC
};

wiz_NetInfo gWIZNETINFO2;*/

uint8_t destination_ip[]={10,10,1,148};
uint16_t destination_port=8001;

void PHYStatusCheck(void)
{
	uint8_t tmp;

	do
	{
		printf("\r\nChecking Ethernet Cable Presence ...");
		ctlwizchip(CW_GET_PHYLINK, (void*) &tmp);

		if(tmp == PHY_LINK_OFF)
		{
			printf("NO Cable Connected!");
			HAL_Delay(1500);
		}
	}while(tmp == PHY_LINK_OFF);

	printf("Good! Cable got connected!");

}


void PrintPHYConf(void)
{
	wiz_PhyConf phyconf;

	ctlwizchip(CW_GET_PHYCONF, (void*) &phyconf);

	if(phyconf.by==PHY_CONFBY_HW)
	{
		printf("\n\rPHY Configured by Hardware Pins");
	}
	else
	{
		printf("\n\rPHY Configured by Registers");
	}

	if(phyconf.mode==PHY_MODE_AUTONEGO)
	{
		printf("\n\rAutonegotiation Enabled");
	}
	else
	{
		printf("\n\rAutonegotiation NOT Enabled");
	}

	if(phyconf.duplex==PHY_DUPLEX_FULL)
	{
		printf("\n\rDuplex Mode: Full");
	}
	else
	{
		printf("\n\rDuplex Mode: Half");
	}

	if(phyconf.speed==PHY_SPEED_10)
	{
		printf("\n\rSpeed: 10Mbps");
	}
	else
	{
		printf("\n\rSpeed: 100Mbps");
	}
}

void WaitMs(UI32_t m)
{
	UI32_t i=0,j=0,k=0;
	for(k=0;k<m;k++)
	{
		for(j=0;j<12;j++)
		{
			for(i=0;i<1000;i++)
			{
			}
		}
	}
}


void Tcp_Msg_send(uint8_t *P)
{
	if(send(1,"",1)<=SOCK_ERROR)
	{
		printf("\r\nSending Failed!");
		//  while(1);
		if((socket(1, Sn_MR_TCP, 0, 0)==1))
		{
			printf("\n\r Socket Created successfully");
		}
		else
		{
			printf("n\rCannot create socket");
			//   while(1);
		}
		// printf("\r\nConnecting to server: %d.%d.%d.%d @ TCP Port: %d",destination_ip[0],destination_ip[1],destination_ip[2],destination_ip[3],destination_port);
		//if(connect(1, destination_ip, destination_port)==SOCK_OK)
		if(connect(1, User_destination_ip, User_destination_port)==SOCK_OK)
		{
			printf("\r\nConnected with server.");
		}
		else
		{
			printf("\r\nCannot connect with server!");
			// while(1);
		}
	}
	else
	{
		printf("\r\nSending Success!");
	}
}


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	// Check which version of the timer triggered this callback and toggle LED
	HAL_TIM_Base_Start_IT(&htim15);
	if (htim == &htim15 )
	{
		g_ulTimer0TickCtr++;
		bln_TimerInterruptFlag1++;
		if(bln_TimerInterruptFlag1>=5)
		{
			//		  printf("TimerE");
			bln_TimerInterruptFlag1= 0;
			bln_TimerInterruptFlag = 1;

			// send(1,"X",1);

		}
		if(g_ulTimer0TickCtr>=500)
		{
			g_ulTimer0TickCtr = 0;
			blnFiveSecondHBTIMER1 = 1;
			bln_ConEstablishedCkeck = 1;

		}

	}
}

void init_sockets() {
	// Set up the TCP communication socket
	// socket(TCP_SOCKET, Sn_MR_TCP, 5000, 0); // Assuming port 5000 for TCP
	//listen(TCP_SOCKET);

	// Set up the UDP discovery socket
	socket(UDP_SOCKET, Sn_MR_UDP, DISCOVERY_PORT, 0);
}

void udp_discovery_service() {
	uint16_t len,valid_request;
	uint8_t remote_ip[4];
	uint16_t remote_port;
	char response_buffer[256];

	// Check for UDP data
	len = getSn_RX_RSR(UDP_SOCKET); // Get received data size for UDP socket
	if (len > 0) {

		if (len > BUFFER_SIZE) len = BUFFER_SIZE; // Cap to buffer size

		// Receive the discovery message
		len = recvfrom(UDP_SOCKET, udp_buffer, len, remote_ip, &remote_port);
		// Respond to discovery requests
		valid_request = strcmp(udp_buffer, "DISCOVERY_REQUEST");
		if(strncmp(udp_buffer, "SET_IP",6)==0)
		{
			blnUart1Printf = 1;
			printf("udp_buffer = %s\n\r",udp_buffer);
			blnUart1Printf = 0;
			// Parse and assign the values
			parse_and_assign(udp_buffer);
			HAL_NVIC_SystemReset(); // Trigger system reset

		}
		memset(udp_buffer,'\0',256);
		//        if (!valid_request) {
		//            const char *response = "DEVICE_INFO:W5500;IP:192.168.1.100;MAC:00:08:DC:12:34:56";
		//            sendto(UDP_SOCKET, (uint8_t *)response, strlen(response), remote_ip, remote_port);
		//        }
		//        {
		//            const char *response = "DEVICE_INFO:W5500;IP:192.168.1.100;MAC:00:08:DC:12:34:56";
		//            sendto(UDP_SOCKET, (uint8_t *)response, strlen(response), remote_ip, remote_port);
		//        }
		if (!valid_request)
		{
			// Format the MAC, IP, Gateway, DNS, Destination IP, and Port number into a single string
			snprintf(response_buffer, sizeof(response_buffer),
					"MAC: %02X:%02X:%02X:%02X:%02X:%02X, "
					"IP: %d.%d.%d.%d, subnet: %d.%d.%d.%d, Gateway: %d.%d.%d.%d, DNS: %d.%d.%d.%d, "
					"Destination IP: %d.%d.%d.%d, Port: %d",
					gWIZNETINFO2.mac[0], gWIZNETINFO2.mac[1], gWIZNETINFO2.mac[2],
					gWIZNETINFO2.mac[3], gWIZNETINFO2.mac[4], gWIZNETINFO2.mac[5],
					gWIZNETINFO2.ip[0], gWIZNETINFO2.ip[1], gWIZNETINFO2.ip[2], gWIZNETINFO2.ip[3],
					gWIZNETINFO2.sn[0], gWIZNETINFO2.sn[1], gWIZNETINFO2.sn[2], gWIZNETINFO2.sn[3],
					gWIZNETINFO2.gw[0], gWIZNETINFO2.gw[1], gWIZNETINFO2.gw[2], gWIZNETINFO2.gw[3],
					gWIZNETINFO2.dns[0], gWIZNETINFO2.dns[1], gWIZNETINFO2.dns[2], gWIZNETINFO2.dns[3],
					User_destination_ip[0], User_destination_ip[1], User_destination_ip[2], User_destination_ip[3],
					User_destination_port);
			// Assign the formatted string to a const char pointer
			const char *response = response_buffer;
			sendto(UDP_SOCKET, (uint8_t *)response, strlen(response), remote_ip, remote_port);
			memset(response_buffer,'\0',256);
		}

	}
}

void checkConnect(void)
{
	if(getSn_SR(1) != SOCK_ESTABLISHED)
	{
		printf("\n\rSOCK_NOT_ESTABLISHED");
		if(socket(1, Sn_MR_TCP, 0, 0)==1)
		{
			printf("\n\r Socket Created successfully");
			Sck_Create_success = 1;
		}
		else
		{
			printf("\n\r Cannot create socket");
		}

		//if((connect(1, destination_ip, destination_port)==SOCK_OK))
		if((connect(1, User_destination_ip, User_destination_port)==SOCK_OK))
		{
			Connected_with_server = 1;
			printf("\n\r Connected with server.");
		}
	}
	else
	{
		printf("\r\nConnected with server!");
	}
}

void checkConnectDefaultIP(void)
{
	if(getSn_SR(1) != SOCK_ESTABLISHED)
	{
		printf("\n\rSOCK_NOT_ESTABLISHED");
		if(socket(1, Sn_MR_TCP, 0, 0)==1)
		{
			printf("\n\r Socket Created successfully");
			Sck_Create_success = 1;
		}
		else
		{
			printf("\n\r Cannot create socket");
		}

		if((connect(1, destination_ip, destination_port)==SOCK_OK))
		{
			Connected_with_server = 1;
			printf("\n\r Connected with server.");
		}
	}
	else
	{
		printf("\r\nConnected with server!");
	}
}



void User_IP_Read(uint16_t fno)
{

	int ct = 0;
	EEPROM_Read(fno, 0, datar4, 30);
	printf("\n\r __datar4 = %s",datar4);
	ct= strlen(datar4);
	datar4[ct++] = '\0';
	//  char input_string[] = "10,1,12,101";
	int array[3];  // Assuming you want to store only the first three numbers in the array
	char *token = strtok((char *)datar4, ",");
	int i = 0;

	while (token != NULL && i <= 4)
	{
		array[i] = atoi(token); // Convert substring to integer and store in the array
		token = strtok(NULL, ",");
		i++;
	}
	// Print the resulting array
	for (int j = 0; j < i; j++)
	{
		if(fno == 4)
		{
			gWIZNETINFO2.ip[j] = array[j];
			printf("\n\r gWIZNETINFO2.ip[%d] = %d", j, gWIZNETINFO2.ip[j]);
		}
		if(fno == 5)
		{
			gWIZNETINFO2.sn[j] = array[j];
			printf("\n\r gWIZNETINFO2.sn[%d] = %d", j, gWIZNETINFO2.sn[j]);
		}
		if(fno == 6)
		{
			gWIZNETINFO2.gw[j] = array[j];
			printf("\n\r gWIZNETINFO2.gw[%d] = %d", j, gWIZNETINFO2.gw[j]);
		}
		if(fno == 7)
		{
			gWIZNETINFO2.dns[j] = array[j];
			printf("\n\r gWIZNETINFO2.dns[%d] = %d", j, gWIZNETINFO2.dns[j]);
		}
		if(fno == 8)
		{
			User_destination_ip[j] = array[j];
			printf("\n\r User_destination_ip[%d] = %d", j, User_destination_ip[j]);
		}
		if(fno == 9 && j==0)
		{
			User_destination_port = array[j];
			printf("\n\r User_destination_port = %d",User_destination_port);
			break;
		}
	}
}
void User_IP_Set(void)
{
	printf("\n\r Trying to upload new IP \n\r");

	//  gWIZNETINFO2 = gWIZNETINFO;
	gWIZNETINFO2.mac[0] = 0x80;
	gWIZNETINFO2.mac[1] = 0x34;
	gWIZNETINFO2.mac[2] = 0x28;
	gWIZNETINFO2.mac[3] = 0x74;
	gWIZNETINFO2.mac[4] = 0xA5;
	gWIZNETINFO2.mac[5] = 0xCB;
	gWIZNETINFO2.dhcp = NETINFO_STATIC;

	blnUart1Printf = 1;
	printf("\n\r ip=%d.%d.%d.%d",gWIZNETINFO2.ip[0],gWIZNETINFO2.ip[1],gWIZNETINFO2.ip[2],gWIZNETINFO2.ip[3]);
	printf("\n\r sn=%d.%d.%d.%d",gWIZNETINFO2.sn[0],gWIZNETINFO2.sn[1],gWIZNETINFO2.sn[2],gWIZNETINFO2.sn[3]);
	printf("\n\r gw=%d.%d.%d.%d",gWIZNETINFO2.gw[0],gWIZNETINFO2.gw[1],gWIZNETINFO2.gw[2],gWIZNETINFO2.gw[3]);
	printf("\n\r dns=%d.%d.%d.%d",gWIZNETINFO2.dns[0],gWIZNETINFO2.dns[1],gWIZNETINFO2.dns[2],gWIZNETINFO2.dns[3]);
	printf("\n\r HostIP=%d.%d.%d.%d",User_destination_ip[0],User_destination_ip[1],User_destination_ip[2],User_destination_ip[3]);
	printf("\n\r HostPort =%d",User_destination_port);
	blnUart1Printf = 0;

	W5500Init();

	ctlnetwork(CN_SET_NETINFO, (void*) &gWIZNETINFO2);

	//configure PHY by software
	wiz_PhyConf phyconf;

	phyconf.by=PHY_CONFBY_SW;
	phyconf.duplex=PHY_DUPLEX_FULL;
	phyconf.speed=PHY_SPEED_10;
	phyconf.mode=PHY_MODE_AUTONEGO;

	ctlwizchip(CW_SET_PHYCONF, (void *) &gWIZNETINFO2);

	PHYStatusCheck();
	PrintPHYConf();



}


void Default_Static_IP_Set(void)
{


	//  , 0x34, 0x28,0x74,0xA5,0xCB;
	W5500Init();

	ctlnetwork(CN_SET_NETINFO, (void*) &gWIZNETINFO);

	//configure PHY by software
	wiz_PhyConf phyconf;

	phyconf.by=PHY_CONFBY_SW;
	phyconf.duplex=PHY_DUPLEX_FULL;
	phyconf.speed=PHY_SPEED_10;
	phyconf.mode=PHY_MODE_AUTONEGO;

	ctlwizchip(CW_SET_PHYCONF, (void *) &gWIZNETINFO);

	PHYStatusCheck();
	PrintPHYConf();

	checkConnectDefaultIP();
	init_sockets();

}


void parse_and_assign(char *received_str) {
	// Temporary buffers for parsing
	char ip_str[16], subnet_str[16], gateway_str[16], dns_str[16], user_ip_str[16],port[10];
	char dataarr[256];

	// Extract values from the string using sscanf
	sscanf(received_str,
			"SET_IP:%15[^;];SubnetMask:%15[^;];Getway:%15[^;];DNS:%15[^;];UserDestiIP:%15[^;];UserDestiPort:%15[^;]",
			ip_str, subnet_str, gateway_str, dns_str, user_ip_str, port);

	sprintf(dataarr,"%s,%s,%s,%s,%s,%s\0",ip_str,subnet_str,gateway_str,dns_str,user_ip_str,port);
	blnUart1Printf = 1;
	printf("dataarr = %s\n\r",dataarr);
	blnUart1Printf = 0;
	//    blnUart1Printf = 1;
	//    printf("ip_str = %s subnet_str=%s gateway_str =%s dns_str=%s user_ip_str=%s\n\r",ip_str,subnet_str,gateway_str,dns_str,user_ip_str);
	//    blnUart1Printf = 0;


	for (int i=0; i<512; i++)
	{
		EEPROM_PageErase(i);
	}
	HAL_Delay(100);



	int array[20];  // Assuming you want to store only the first three numbers in the array// 3*4=12
	char *token = strtok((char *)dataarr, ". ,");
	int i = 0;

	while (token != NULL && i <= 20)  //Chnage 15 to 19
	{
		array[i] = atoi(token); // Convert substring to integer and store in the array
		token = strtok(NULL, ". ,");
		i++;
	}
	for (int j = 0; j < i; j++) {
		printf("\t array[%d] = %d", j, array[j]);
	}

	// printf("\n\r User Entered IP = %d",gWIZNETINFO2.ip);

	/**********************************IP START************************************************/
	// int array[] = {10, 10, 1, 101};
	int n = sizeof(array) / sizeof(array[0]);

	// Convert array elements to char string
	char charString[50]; // Assuming a maximum length of 100 characters for the resulting string
	int offset = 0;
	i=0;

	// Loop through the array and concatenate elements with a comma separator
	for (i = 0; i < 4; i++) {
		offset += snprintf(charString + offset, sizeof(charString) - offset, "%d", array[i]);
		if (i < 4 - 1) {
			offset += snprintf(charString + offset, sizeof(charString) - offset, ",");
		}
		else
		{
			offset += snprintf(charString + offset, sizeof(charString) - offset, '\0');
		}
	}

	// Print the resulting char string
	printf("\n\rcharString =%s", charString);
	/**********************************IP END************************************************/

	/**********************************Sn START************************************************/
	char charString1[50];
	offset = 0;

	for (i = 4; i < 8; i++) {
		offset += snprintf(charString1 + offset, sizeof(charString1) - offset, "%d", array[i]);
		if (i < 8 - 1) {
			offset += snprintf(charString1 + offset, sizeof(charString1) - offset, ",");
		}
		else
		{
			offset += snprintf(charString1 + offset, sizeof(charString1) - offset, '\0');
		}
	}

	printf("\n\r!charString1 =%s", charString1);
	/**********************************Sn END ********************************************************/

	/**********************************Gw START************************************************/
	char charString2[50];
	offset = 0;

	for (i = 8; i < 12; i++) {
		offset += snprintf(charString2 + offset, sizeof(charString2) - offset, "%d", array[i]);
		if (i < 12 - 1) {
			offset += snprintf(charString2 + offset, sizeof(charString2) - offset, ",");
		}
		else
		{
			offset += snprintf(charString2 + offset, sizeof(charString2) - offset, '\0');
		}
	}

	printf("\n\r!charString2 =%s", charString2);
	/**********************************Gw END ********************************************************/

	/**********************************Dns START************************************************/
	char charString3[50];
	offset = 0;

	for (i = 12; i < 16; i++) {
		offset += snprintf(charString3 + offset, sizeof(charString3) - offset, "%d", array[i]);
		if (i < 16 - 1) {
			offset += snprintf(charString3 + offset, sizeof(charString3) - offset, ",");
		}
		else
		{
			//   offset += snprintf(charString3 + offset, sizeof(charString3) - offset, '\0');
		}
	}

	printf("\n\r!charString3 =%s", charString3);
	/**********************************Dns END ********************************************************/

	/**********************************Distination IP START********************************************/
	char charString4[50];
	offset = 0;

	for (i = 16; i < 20; i++) {
		offset += snprintf(charString4 + offset, sizeof(charString4) - offset, "%d", array[i]);
		if (i < 20 - 1) {
			offset += snprintf(charString4 + offset, sizeof(charString4) - offset, ",");
		}
		else
		{
			offset += snprintf(charString4 + offset, sizeof(charString4) - offset, '\0');
		}
	}

	printf("\n\r!charString4 =%s", charString4);
	/**********************************Distination IP  END ********************************************************/
	char charString5[10];
	offset = 0;

	for (i = 20; i < 21; i++) {
		offset += snprintf(charString5 + offset, sizeof(charString5) - offset, "%d", array[i]);
		if (i < 20 - 1) {
			offset += snprintf(charString5 + offset, sizeof(charString5) - offset, ",");
		}
		else{
			offset += snprintf(charString5 + offset, sizeof(charString5) - offset, '\0');
		}
	}

	printf("\n\r!charString5 =%s", charString5);

	gWIZNETINFO2.ip[0] = array[0];
	gWIZNETINFO2.ip[1] = array[1];
	gWIZNETINFO2.ip[2] = array[2];
	gWIZNETINFO2.ip[3] = array[3];

	gWIZNETINFO2.sn[0] = array[4];
	gWIZNETINFO2.sn[1] = array[5];
	gWIZNETINFO2.sn[2] = array[6];
	gWIZNETINFO2.sn[3] = array[7];

	// printf("\n\r User Entered Sn = %s",gWIZNETINFO2.sn);

	gWIZNETINFO2.gw[0] = array[8];
	gWIZNETINFO2.gw[1] = array[9];
	gWIZNETINFO2.gw[2] = array[10];
	gWIZNETINFO2.gw[3] = array[11];

	// printf("\n\r User Entered GW = %s",gWIZNETINFO2.gw);

	gWIZNETINFO2.dns[0] = array[12];
	gWIZNETINFO2.dns[1] = array[13];
	gWIZNETINFO2.dns[2] = array[14];
	gWIZNETINFO2.dns[3] = array[15];

	// printf("\n\r User Entered DNS = %s",gWIZNETINFO2.dns);
	User_destination_ip[0] = array[16];
	User_destination_ip[1] = array[17];
	User_destination_ip[2] = array[18];
	User_destination_ip[3] = array[19];
	User_destination_port= array[20];

	// printf("\n\r User Entered DNS = %s",gWIZNETINFO2.dns);

	printf("\n\r ip=%d.%d.%d.%d",gWIZNETINFO2.ip[0],gWIZNETINFO2.ip[1],gWIZNETINFO2.ip[2],gWIZNETINFO2.ip[3]);
	printf("\n\r sn=%d.%d.%d.%d",gWIZNETINFO2.sn[0],gWIZNETINFO2.sn[1],gWIZNETINFO2.sn[2],gWIZNETINFO2.sn[3]);
	printf("\n\r gw=%d.%d.%d.%d",gWIZNETINFO2.gw[0],gWIZNETINFO2.gw[1],gWIZNETINFO2.gw[2],gWIZNETINFO2.gw[3]);
	printf("\n\r dns=%d.%d.%d.%d",gWIZNETINFO2.dns[0],gWIZNETINFO2.dns[1],gWIZNETINFO2.dns[2],gWIZNETINFO2.dns[3]);
	printf("\n\r HostIP=%d.%d.%d.%d",User_destination_ip[0],User_destination_ip[1],User_destination_ip[2],User_destination_ip[3]);
	printf("\n\r HOSTPost=%d",User_destination_port);

	for (int i=0; i<512; i++)
	{
		EEPROM_PageErase(i);
	}
	HAL_Delay(100);

	gWIZNETINFO2.mac[0] = 0x80;
	gWIZNETINFO2.mac[1] = 0x34;
	gWIZNETINFO2.mac[2] = 0x28;
	gWIZNETINFO2.mac[3] = 0x74;
	gWIZNETINFO2.mac[4] = 0xA5;
	gWIZNETINFO2.mac[5] = 0xCB;
	gWIZNETINFO2.dhcp = NETINFO_STATIC;
	EEPROM_Write(3, 0, "0x80, 0x34, 0x28,0x74,0xA5,0xCB", strlen((char *)"0x80, 0x34, 0x28,0x74,0xA5,0xCB"));

	EEPROM_Write(4, 0, charString,strlen(charString));
	EEPROM_Write(5, 0, charString1,strlen(charString1));
	EEPROM_Write(6, 0, charString2,strlen(charString2));
	EEPROM_Write(7, 0, charString3,strlen(charString3));
	EEPROM_Write(8, 0, charString4, strlen(charString4));
	EEPROM_Write(9, 0, charString5, strlen(charString5));

	/*				EEPROM_Write(4, 0, "10,10,1,210\0", strlen((char *)"10,10,1,210\0"));
					EEPROM_Write(5, 0, "255,0,0,0", strlen((char *)"255,0,0,0"));
					EEPROM_Write(6, 0, "10,10,1,254", strlen((char *)"10,10,1,254"));
					EEPROM_Write(7, 0, "10,10,1,254", strlen((char *)"10,10,1,254"));*/

	HAL_Delay(100);

	/*				printf("\n\r @ip=%d.%d.%d.%d",gWIZNETINFO2.ip[0],gWIZNETINFO2.ip[1],gWIZNETINFO2.ip[2],gWIZNETINFO2.ip[3]);
					printf("\n\r sn=%d.%d.%d.%d",gWIZNETINFO2.sn[0],gWIZNETINFO2.sn[1],gWIZNETINFO2.sn[2],gWIZNETINFO2.sn[3]);
					printf("\n\r gw=%d.%d.%d.%d",gWIZNETINFO2.gw[0],gWIZNETINFO2.gw[1],gWIZNETINFO2.gw[2],gWIZNETINFO2.gw[3]);
					printf("\n\r dns=%d.%d.%d.%d",gWIZNETINFO2.dns[0],gWIZNETINFO2.dns[1],gWIZNETINFO2.dns[2],gWIZNETINFO2.dns[3]);
					printf("\n\r HostIP=%d.%d.%d.%d",User_destination_ip[0],User_destination_ip[1],User_destination_ip[2],User_destination_ip[3]);*/
	W5500Init();

	ctlnetwork(CN_SET_NETINFO, (void*) &gWIZNETINFO2);

	//configure PHY by software
	wiz_PhyConf phyconf;

	phyconf.by=PHY_CONFBY_SW;
	phyconf.duplex=PHY_DUPLEX_FULL;
	phyconf.speed=PHY_SPEED_10;
	phyconf.mode=PHY_MODE_AUTONEGO;

	ctlwizchip(CW_SET_PHYCONF, (void *) &gWIZNETINFO2);

	PHYStatusCheck();
	PrintPHYConf();


	memset(U0_RevLocl_Buf,'\0',256);
	memset(receive_buff,'\0',256);
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */
	uint16_t timer_val;
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
	// w5500Init();
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_USART1_UART_Init();
  MX_USART2_UART_Init();
  MX_ADC1_Init();
  MX_TIM15_Init();
  MX_SPI1_Init();
  MX_I2C2_Init();
  /* USER CODE BEGIN 2 */



	HAL_UART_Transmit(&huart1, Rx1Buffer, 1,2);
	HAL_UART_Transmit(&huart2, Rx1Buffer, 1,2);
	HAL_UART_Receive_IT(&huart1, Rx1_data,1);
	HAL_UART_Receive_IT(&huart2,Rx2_data,1);
	HAL_ADC_Start(&hadc1);

	printf("A Simple TCP Client Application using w5500\n\r");

	User_IP_Read(4);
	User_IP_Read(5);
	User_IP_Read(6);
	User_IP_Read(7);
	User_IP_Read(8);
	User_IP_Read(9);
	User_IP_Set();
	init_sockets();     // Initialize sockets

	/*
   W5500Init();

   ctlnetwork(CN_SET_NETINFO, (void*) &gWIZNETINFO);

   //configure PHY by software
   wiz_PhyConf phyconf;

   phyconf.by=PHY_CONFBY_SW;
   phyconf.duplex=PHY_DUPLEX_FULL;
   phyconf.speed=PHY_SPEED_10;
   phyconf.mode=PHY_MODE_AUTONEGO;

   ctlwizchip(CW_SET_PHYCONF, (void *) &gWIZNETINFO);

   PHYStatusCheck();
   PrintPHYConf();
	 */

	HAL_TIM_Base_Start_IT(&htim15);
	timer_val = __HAL_TIM_GET_COUNTER(&htim15);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1)
	{
		udp_discovery_service();  // Handle UDP discovery requests

		if(bln_ConEstablishChk_ser)
		{
			if(bln_ConEstablishedCkeck)
			{
				checkConnectDefaultIP();
				bln_ConEstablishedCkeck = 0;
			}
		}
		/*************** Connection Established check function START **********/
		if(bln_ConEstablishedCkeck  && !bln_ConEstablishChk_ser)
		{
			// Tcp_Msg_send("X");
			checkConnect();
			bln_ConEstablishedCkeck = 0;
		}
		/*************** Connection Established check function END **********/


		/*************** Check Ethernet data Receive function START **********/
		len = getSn_RX_RSR(1);
		if(len)
		{
			len=recv(1, receive_buff, RECEIVE_BUFF_SIZE);
			receive_buff[len]='\0';
			blnUart1Printf = 1;
			printf("%s", receive_buff);
			blnUart1Printf = 0;
			printf("%s", receive_buff);
			len = 0;
		}
		/*************** Check Ethernet data Receive function END **********/

		/*************** Default IP set request function START **********/
		if(blnFiveSecondHBTIMER1)
		{
			if(HAL_GPIO_ReadPin(GPIOC, IN1_Pin)== LOW)
			{
				g_ulT0w5500RstCtr++;
				if(g_ulT0w5500RstCtr > 2)
				{
					bln_ConEstablishChk_ser = 1;
					printf("Default IP Command Received");
					Default_Static_IP_Set();
					g_ulT0w5500RstCtr = 0;
				}
			}
			else
			{
				g_ulT0w5500RstCtr = 0;
			}
			blnFiveSecondHBTIMER1 = 0;
		}
		/*************** Default ip set request function END ***********/

		/*********************User New IP set Command Function Start FromUart0 *******************************/

		if(strcmp(U0_RevLocl_Buf,"PleaseSetNewIP:")==0 || strcmp(receive_buff,"PleaseSetNewIP:")==0)    //Uart0 Data received
		{
			printf("\n\rPlease enter your ID");
			len=recv(1, receive_buff, RECEIVE_BUFF_SIZE);
			receive_buff[len]='\0';
			printf("\n\r your Enter data is =%s\n\r",receive_buff);

			int array[20];  // Assuming you want to store only the first three numbers in the array// 3*4=12
			char *token = strtok((char *)receive_buff, ". ,");
			int i = 0;

			while (token != NULL && i <= 20)  //Chnage 15 to 19
			{
				array[i] = atoi(token); // Convert substring to integer and store in the array
				token = strtok(NULL, ". ,");
				i++;
			}
			for (int j = 0; j < i; j++) {
				printf("\t array[%d] = %d", j, array[j]);
			}
			gWIZNETINFO2.ip[0] = array[0];
			gWIZNETINFO2.ip[1] = array[1];
			gWIZNETINFO2.ip[2] = array[2];
			gWIZNETINFO2.ip[3] = array[3];

			// printf("\n\r User Entered IP = %d",gWIZNETINFO2.ip);

			/**********************************IP START************************************************/
			// Convert array elements to char string
			char charString[50]; // Assuming a maximum length of 100 characters for the resulting string
			int offset = 0;
			i=0;

			// Loop through the array and concatenate elements with a comma separator
			for (i = 0; i < 4; i++) {
				offset += snprintf(charString + offset, sizeof(charString) - offset, "%d", array[i]);
				if (i < 4 - 1) {
					offset += snprintf(charString + offset, sizeof(charString) - offset, ",");
				}
				else
				{
					offset += snprintf(charString + offset, sizeof(charString) - offset, '\0');
				}
			}

			// Print the resulting char string
			printf("\n\rcharString =%s", charString);
			/**********************************IP END************************************************/

			/**********************************Sn START************************************************/
			char charString1[50];
			offset = 0;

			for (i = 4; i < 8; i++) {
				offset += snprintf(charString1 + offset, sizeof(charString1) - offset, "%d", array[i]);
				if (i < 8 - 1) {
					offset += snprintf(charString1 + offset, sizeof(charString1) - offset, ",");
				}
				else
				{
					offset += snprintf(charString1 + offset, sizeof(charString1) - offset, '\0');
				}
			}

			printf("\n\r!charString1 =%s", charString1);
			/**********************************Sn END ********************************************************/

			/**********************************Gw START************************************************/
			char charString2[50];
			offset = 0;

			for (i = 8; i < 12; i++) {
				offset += snprintf(charString2 + offset, sizeof(charString2) - offset, "%d", array[i]);
				if (i < 12 - 1) {
					offset += snprintf(charString2 + offset, sizeof(charString2) - offset, ",");
				}
				else
				{
					offset += snprintf(charString2 + offset, sizeof(charString2) - offset, '\0');
				}
			}

			printf("\n\r!charString2 =%s", charString2);
			/**********************************Gw END ********************************************************/

			/**********************************Dns START************************************************/
			char charString3[50];
			offset = 0;

			for (i = 12; i < 16; i++) {
				offset += snprintf(charString3 + offset, sizeof(charString3) - offset, "%d", array[i]);
				if (i < 16 - 1) {
					offset += snprintf(charString3 + offset, sizeof(charString3) - offset, ",");
				}
				else
				{
					//   offset += snprintf(charString3 + offset, sizeof(charString3) - offset, '\0');
				}
			}

			printf("\n\r!charString3 =%s", charString3);
			/**********************************Dns END ********************************************************/

			/**********************************Distination IP START********************************************/
			char charString4[50];
			offset = 0;

			for (i = 16; i < 20; i++) {
				offset += snprintf(charString4 + offset, sizeof(charString4) - offset, "%d", array[i]);
				if (i < 20 - 1) {
					offset += snprintf(charString4 + offset, sizeof(charString4) - offset, ",");
				}
				else
				{
					offset += snprintf(charString4 + offset, sizeof(charString4) - offset, '\0');
				}
			}

			printf("\n\r!charString4 =%s", charString4);
			/**********************************Distination IP  END ********************************************************/
			char charString5[10];
			offset = 0;

			for (i = 20; i < 21; i++) {
				offset += snprintf(charString5 + offset, sizeof(charString5) - offset, "%d", array[i]);
				if (i < 20 - 1) {
					offset += snprintf(charString5 + offset, sizeof(charString5) - offset, ",");
				}
				else{
					offset += snprintf(charString5 + offset, sizeof(charString5) - offset, '\0');
				}
			}

			printf("\n\r!charString5 =%s", charString5);


			gWIZNETINFO2.sn[0] = array[4];
			gWIZNETINFO2.sn[1] = array[5];
			gWIZNETINFO2.sn[2] = array[6];
			gWIZNETINFO2.sn[3] = array[7];

			// printf("\n\r User Entered Sn = %s",gWIZNETINFO2.sn);

			gWIZNETINFO2.gw[0] = array[8];
			gWIZNETINFO2.gw[1] = array[9];
			gWIZNETINFO2.gw[2] = array[10];
			gWIZNETINFO2.gw[3] = array[11];

			// printf("\n\r User Entered GW = %s",gWIZNETINFO2.gw);

			gWIZNETINFO2.dns[0] = array[12];
			gWIZNETINFO2.dns[1] = array[13];
			gWIZNETINFO2.dns[2] = array[14];
			gWIZNETINFO2.dns[3] = array[15];

			// printf("\n\r User Entered DNS = %s",gWIZNETINFO2.dns);
			User_destination_ip[0] = array[16];
			User_destination_ip[1] = array[17];
			User_destination_ip[2] = array[18];
			User_destination_ip[3] = array[19];
			User_destination_port= array[20];

			// printf("\n\r User Entered DNS = %s",gWIZNETINFO2.dns);

			printf("\n\r ip=%d.%d.%d.%d",gWIZNETINFO2.ip[0],gWIZNETINFO2.ip[1],gWIZNETINFO2.ip[2],gWIZNETINFO2.ip[3]);
			printf("\n\r sn=%d.%d.%d.%d",gWIZNETINFO2.sn[0],gWIZNETINFO2.sn[1],gWIZNETINFO2.sn[2],gWIZNETINFO2.sn[3]);
			printf("\n\r gw=%d.%d.%d.%d",gWIZNETINFO2.gw[0],gWIZNETINFO2.gw[1],gWIZNETINFO2.gw[2],gWIZNETINFO2.gw[3]);
			printf("\n\r dns=%d.%d.%d.%d",gWIZNETINFO2.dns[0],gWIZNETINFO2.dns[1],gWIZNETINFO2.dns[2],gWIZNETINFO2.dns[3]);
			printf("\n\r HostIP=%d.%d.%d.%d",User_destination_ip[0],User_destination_ip[1],User_destination_ip[2],User_destination_ip[3]);
			printf("\n\r HOSTPost=%d",User_destination_port);

			for (int i=0; i<512; i++)
			{
				EEPROM_PageErase(i);
			}
			HAL_Delay(100);

			gWIZNETINFO2.mac[0] = 0x80;
			gWIZNETINFO2.mac[1] = 0x34;
			gWIZNETINFO2.mac[2] = 0x28;
			gWIZNETINFO2.mac[3] = 0x74;
			gWIZNETINFO2.mac[4] = 0xA5;
			gWIZNETINFO2.mac[5] = 0xCB;
			gWIZNETINFO2.dhcp = NETINFO_STATIC;
			EEPROM_Write(3, 0, "0x80, 0x34, 0x28,0x74,0xA5,0xCB", strlen((char *)"0x80, 0x34, 0x28,0x74,0xA5,0xCB"));

			EEPROM_Write(4, 0, charString,strlen(charString3));
			EEPROM_Write(5, 0, charString1,strlen(charString1));
			EEPROM_Write(6, 0, charString2,strlen(charString2));
			EEPROM_Write(7, 0, charString3,strlen(charString3));
			EEPROM_Write(8, 0, charString4, strlen(charString4));
			EEPROM_Write(9, 0, charString5, strlen(charString5));

			HAL_Delay(100);

			/*				printf("\n\r @ip=%d.%d.%d.%d",gWIZNETINFO2.ip[0],gWIZNETINFO2.ip[1],gWIZNETINFO2.ip[2],gWIZNETINFO2.ip[3]);
				printf("\n\r sn=%d.%d.%d.%d",gWIZNETINFO2.sn[0],gWIZNETINFO2.sn[1],gWIZNETINFO2.sn[2],gWIZNETINFO2.sn[3]);
				printf("\n\r gw=%d.%d.%d.%d",gWIZNETINFO2.gw[0],gWIZNETINFO2.gw[1],gWIZNETINFO2.gw[2],gWIZNETINFO2.gw[3]);
				printf("\n\r dns=%d.%d.%d.%d",gWIZNETINFO2.dns[0],gWIZNETINFO2.dns[1],gWIZNETINFO2.dns[2],gWIZNETINFO2.dns[3]);
				printf("\n\r HostIP=%d.%d.%d.%d",User_destination_ip[0],User_destination_ip[1],User_destination_ip[2],User_destination_ip[3]);*/
			W5500Init();

			ctlnetwork(CN_SET_NETINFO, (void*) &gWIZNETINFO2);

			//configure PHY by software
			wiz_PhyConf phyconf;

			phyconf.by=PHY_CONFBY_SW;
			phyconf.duplex=PHY_DUPLEX_FULL;
			phyconf.speed=PHY_SPEED_10;
			phyconf.mode=PHY_MODE_AUTONEGO;

			ctlwizchip(CW_SET_PHYCONF, (void *) &gWIZNETINFO2);

			PHYStatusCheck();
			PrintPHYConf();


			memset(U0_RevLocl_Buf,'\0',256);
			memset(receive_buff,'\0',256);

			/*********************User New IP set Command Function END  FromUart0*******************************/

		}

    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	}
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 10;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV7;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc1.Init.Resolution = ADC_RESOLUTION_10B;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
  hadc1.Init.OversamplingMode = DISABLE;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_6;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_2CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.Timing = 0x007074AF;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_2;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */
	__HAL_SPI_ENABLE(&hspi1);
  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief TIM15 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM15_Init(void)
{

  /* USER CODE BEGIN TIM15_Init 0 */

  /* USER CODE END TIM15_Init 0 */

  TIM_SlaveConfigTypeDef sSlaveConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM15_Init 1 */

  /* USER CODE END TIM15_Init 1 */
  htim15.Instance = TIM15;
  htim15.Init.Prescaler = 1;
  htim15.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim15.Init.Period = 65535;
  htim15.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim15.Init.RepetitionCounter = 0;
  htim15.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim15) != HAL_OK)
  {
    Error_Handler();
  }
  sSlaveConfig.SlaveMode = TIM_SLAVEMODE_RESET;
  sSlaveConfig.InputTrigger = TIM_TS_ITR0;
  if (HAL_TIM_SlaveConfigSynchro(&htim15, &sSlaveConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim15, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM15_Init 2 */

  /* USER CODE END TIM15_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 9600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 9600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel2_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel2_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel2_IRQn);
  /* DMA1_Channel5_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel5_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel5_IRQn);
  /* DMA1_Channel6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel6_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, cs_Pin|Rst_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : IN1_Pin */
  GPIO_InitStruct.Pin = IN1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(IN1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : cs_Pin Rst_Pin */
  GPIO_InitStruct.Pin = cs_Pin|Rst_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1)
	{
	}
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
	/* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
