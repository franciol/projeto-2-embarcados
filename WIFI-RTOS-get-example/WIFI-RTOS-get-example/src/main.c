#include "asf.h"
#include "main.h"
#include <string.h>
#include "bsp/include/nm_bsp.h"
#include "driver/include/m2m_wifi.h"
#include "socket/include/socket.h"

#define STRING_EOL "\r\n"
#define STRING_HEADER "-- WINC1500 weather client example --" STRING_EOL \
					  "-- " BOARD_NAME " --" STRING_EOL                  \
					  "-- Compiled: "__DATE__                            \
					  " "__TIME__                                        \
					  " --" STRING_EOL

#define BUZZ_PIO PIOC
#define BUZZ_PIO_ID ID_PIOC
#define BUZZ_PIO_IDX 13u
#define BUZZ_PIO_IDX_MASK (1u << BUZZ_PIO_IDX)

#define DOWN_PIO PIOD
#define DOWN_PIO_ID ID_PIOD
#define DOWN_PIO_IDX 30u
#define DOWN_MASK (1u << DOWN_PIO_IDX)

SemaphoreHandle_t xSemaphore, xSemaphoreWIFI;
QueueHandle_t xQueuePWM, xQueuePWMDOWN;

/** MODE  */
bool DOWNLOAD_MODE = false;
bool OFF_M = false;

/** VALUES PING */
uint32_t pre_ms, pos_ms, ping_ms, sum_ping, counting_ping = 0;

/** IP address of host. */
uint32_t gu32HostIp = 0;

/** TCP client socket handlers. */
static SOCKET tcp_client_socket = -1;

/** Receive buffer definition. */
static uint8_t gau8ReceivedBuffer[MAIN_WIFI_M2M_BUFFER_SIZE] = {0};

/** Wi-Fi status variable. */
static bool gbConnectedWifi = false;

/** Get host IP status variable. */
/** Wi-Fi connection state */
static uint8_t wifi_connected;

/** Instance of HTTP client module. */
static bool gbHostIpByName = false;

/** TCP Connection status variable. */
static bool gbTcpConnection = false;

/** Server host name. */
static char server_host_name[] = MAIN_SERVER_NAME;

#define TASK_WIFI_STACK_SIZE (4096 / sizeof(portSTACK_TYPE))
#define TASK_WIFI_STACK_PRIORITY (tskIDLE_PRIORITY)
#define TASK_BUZZER_STACK_SIZE (4096 / sizeof(portSTACK_TYPE))
#define TASK_BUZZER_STACK_PRIORITY (tskIDLE_PRIORITY)

extern void vApplicationStackOverflowHook(xTaskHandle *pxTask,
										  signed char *pcTaskName);
extern void vApplicationIdleHook(void);
extern void vApplicationTickHook(void);
extern void vApplicationMallocFailedHook(void);
extern void xPortSysTickHandler(void);

/**
 * \brief Called if stack overflow during execution
 */
extern void vApplicationStackOverflowHook(xTaskHandle *pxTask,
										  signed char *pcTaskName)
{
	printf("stack overflow %x %s\r\n", pxTask, (portCHAR *)pcTaskName);
	/* If the parameters have been corrupted then inspect pxCurrentTCB to
	 * identify which task has overflowed its stack.
	 */
	for (;;)
	{
	}
}

/**
 * \brief This function is called by FreeRTOS idle task
 */
extern void vApplicationIdleHook(void)
{
}

/**
 * \brief This function is called by FreeRTOS each tick
 */
extern void vApplicationTickHook(void)
{
}

extern void vApplicationMallocFailedHook(void)
{
	/* Called if a call to pvPortMalloc() fails because there is insufficient
	free memory available in the FreeRTOS heap.  pvPortMalloc() is called
	internally by FreeRTOS API functions that create tasks, queues, software
	timers, and semaphores.  The size of the FreeRTOS heap is set by the
	configTOTAL_HEAP_SIZE configuration constant in FreeRTOSConfig.h. */

	/* Force an assert. */
	configASSERT((volatile void *)NULL);
}

static void RTT_init(uint16_t pllPreScale, uint32_t IrqNPulses)
{
	uint32_t ul_previous_time;

	/* Configure RTT for a 1 second tick interrupt */
	rtt_sel_source(RTT, false);
	rtt_init(RTT, pllPreScale);

	ul_previous_time = rtt_read_timer_value(RTT);
	while (ul_previous_time == rtt_read_timer_value(RTT))
		;
}

/**
 * \brief Configure UART console.
 */
static void configure_console(void)
{
	const usart_serial_options_t uart_serial_options = {
		.baudrate = CONF_UART_BAUDRATE,
		.charlength = CONF_UART_CHAR_LENGTH,
		.paritytype = CONF_UART_PARITY,
		.stopbits = CONF_UART_STOP_BITS,
	};

	/* Configure UART console. */
	sysclk_enable_peripheral_clock(CONSOLE_UART_ID);
	stdio_serial_init(CONF_UART, &uart_serial_options);
}

/* 
 * Check whether "cp" is a valid ascii representation
 * of an Internet address and convert to a binary address.
 * Returns 1 if the address is valid, 0 if not.
 * This replaces inet_addr, the return value from which
 * cannot distinguish between failure and a local broadcast address.
 */
/* http://www.cs.cmu.edu/afs/cs/academic/class/15213-f00/unpv12e/libfree/inet_aton.c */
int inet_aton(const char *cp, in_addr *ap)
{
	int dots = 0;
	register u_long acc = 0, addr = 0;

	do
	{
		register char cc = *cp;

		switch (cc)
		{
		case '0':
		case '1':
		case '2':
		case '3':
		case '4':
		case '5':
		case '6':
		case '7':
		case '8':
		case '9':
			acc = acc * 10 + (cc - '0');
			break;

		case '.':
			if (++dots > 3)
			{
				return 0;
			}
			/* Fall through */

		case '\0':
			if (acc > 255)
			{
				return 0;
			}
			addr = addr << 8 | acc;
			acc = 0;
			break;

		default:
			return 0;
		}
	} while (*cp++);

	/* Normalize the address */
	if (dots < 3)
	{
		addr <<= 8 * (3 - dots);
	}

	/* Store it if requested */
	if (ap)
	{
		ap->s_addr = _htonl(addr);
	}

	return 1;
}

/**
 * \brief Callback function of IP address.
 *
 * \param[in] hostName Domain name.
 * \param[in] hostIp Server IP.
 *
 * \return None.
 */
static void resolve_cb(uint8_t *hostName, uint32_t hostIp)
{
	gu32HostIp = hostIp;
	gbHostIpByName = true;
	printf("resolve_cb: %s IP address is %d.%d.%d.%d\r\n\r\n", hostName,
		   (int)IPV4_BYTE(hostIp, 0), (int)IPV4_BYTE(hostIp, 1),
		   (int)IPV4_BYTE(hostIp, 2), (int)IPV4_BYTE(hostIp, 3));
}

static void task_pwm(void *pvParameters)
{
	uint32_t pos, last = 0;
	uint32_t recD;
	xQueuePWMDOWN = xQueueCreate(10, sizeof(uint32_t));

	pio_set_output(DOWN_PIO, DOWN_MASK, 0, 0, 0);

	for (int i = 0; i < 100; i++)
	{
		pio_clear(DOWN_PIO, DOWN_MASK);
		delay_us(18000);
		pio_set(DOWN_PIO, DOWN_MASK);
		delay_us(2000);
	}

	for (;;)
	{

		if (xQueueReceive(xQueuePWMDOWN, &recD, 0) == pdTRUE)
		{
			//DOWNLOAD
			last = (uint32_t)(recD);
char teste[128];
			sprintf(teste, "DOWNLOAD TIME: %d kbps\n", last);
			printf(teste);
			last = ((last-167400)*5)+1000;
			if (last < 1000)
			{
				last = 1000;
			}
			else if (last > 2000)
			{
				last = 2000;
			}
			sprintf(teste, "DOWNLOAD TIME2: %d kbps\n", last);
			printf(teste);

			
			for (int i = 0; i < 25; i++)
			{
				pio_clear(DOWN_PIO, DOWN_MASK);
				delay_us(20000 - (3000-last));
				pio_set(DOWN_PIO, DOWN_MASK);
				delay_us(3000-last);
			}
		}
		vTaskDelay((TickType_t)500 / portTICK_PERIOD_MS);
	}
}

/**
 * \brief Callback function of TCP client socket.
 *
 * \param[in] sock socket handler.
 * \param[in] u8Msg Type of Socket notification
 * \param[in] pvMsg A structure contains notification informations.
 *
 * \return None.
 */
static void socket_cb(SOCKET sock, uint8_t u8Msg, void *pvMsg)
{

	/* Check for socket event on TCP socket. */
	if (sock == tcp_client_socket)
	{

		switch (u8Msg)
		{
		case SOCKET_MSG_CONNECT:
		{
			//printf("socket_msg_connect\n");
			if (gbTcpConnection)
			{

				//printf("DOWNLOAD_MODE\n");

				memset(gau8ReceivedBuffer, 0, sizeof(gau8ReceivedBuffer));
				sprintf((char *)gau8ReceivedBuffer, "%s", MAIN_PREFIX_BUFFER_DOWN);

				tstrSocketConnectMsg *pstrConnect = (tstrSocketConnectMsg *)pvMsg;
				if (pstrConnect && pstrConnect->s8Error >= SOCK_ERR_NO_ERROR)
				{

					//printf("send\n");
					send(tcp_client_socket, gau8ReceivedBuffer, strlen((char *)gau8ReceivedBuffer), 0);
					memset(gau8ReceivedBuffer, 0, MAIN_WIFI_M2M_BUFFER_SIZE);
					recv(tcp_client_socket, &gau8ReceivedBuffer[0], MAIN_WIFI_M2M_BUFFER_SIZE, 0);
				}
				else
				{
					//printf("socket_cb: connect error!\r\n");
					gbTcpConnection = false;
					close(tcp_client_socket);
					tcp_client_socket = -1;
				}
			}
		}
		break;

		case SOCKET_MSG_RECV:
		{

			char *pcIndxPtr;
			char *pcEndPtr;

			tstrSocketRecvMsg *pstrRecv = (tstrSocketRecvMsg *)pvMsg;
			if (pstrRecv && pstrRecv->s16BufferSize > 0)
			{
				uint32_t send = rtt_read_timer_value(RTT) - pre_ms;
				
				// printf("--------------------------------------\n");
				//printf("---> %d \n\n", pstrRecv->u16RemainingSize);
			//	printf(pstrRecv->pu8Buffer);
				//printf("END-----------------------------------\n");

				uint8 *pEND = strstr(pstrRecv->pu8Buffer, "00000");
				//printf("%d", pEND);
				if(pEND){

					uint8 size = pEND - pstrRecv->pu8Buffer;
					uint8 speed = size/(send);
					if(speed>0){						
						xQueueSend(xQueuePWMDOWN, &speed, 0);}
				}

				memset(gau8ReceivedBuffer, 0, sizeof(gau8ReceivedBuffer));
				
				pre_ms = rtt_read_timer_value(RTT);	
				recv(tcp_client_socket, &gau8ReceivedBuffer[0], MAIN_WIFI_M2M_BUFFER_SIZE, 0);
				
				// vTaskDelay((TickType_t)100 / portTICK_PERIOD_MS);
			}
			else
			{
				//printf("socket_cb: recv error!\r\n");
				close(tcp_client_socket);
				tcp_client_socket = -1;
				pre_ms = rtt_read_timer_value(RTT);
				pos_ms = rtt_read_timer_value(RTT);
			}
		}
		break;

		default:
			break;
		}
	}
}

static void set_dev_name_to_mac(uint8_t *name, uint8_t *mac_addr)
{
	/* Name must be in the format WINC1500_00:00 */
	uint16 len;

	len = m2m_strlen(name);
	if (len >= 5)
	{
		name[len - 1] = MAIN_HEX2ASCII((mac_addr[5] >> 0) & 0x0f);
		name[len - 2] = MAIN_HEX2ASCII((mac_addr[5] >> 4) & 0x0f);
		name[len - 4] = MAIN_HEX2ASCII((mac_addr[4] >> 0) & 0x0f);
		name[len - 5] = MAIN_HEX2ASCII((mac_addr[4] >> 4) & 0x0f);
	}
}

/**
 * \brief Callback to get the Wi-Fi status update.
 *
 * \param[in] u8MsgType Type of Wi-Fi notification.
 * \param[in] pvMsg A pointer to a buffer containing the notification parameters.
 *
 * \return None.
 */
static void wifi_cb(uint8_t u8MsgType, void *pvMsg)
{
	switch (u8MsgType)
	{
	case M2M_WIFI_RESP_CON_STATE_CHANGED:
	{
		tstrM2mWifiStateChanged *pstrWifiState = (tstrM2mWifiStateChanged *)pvMsg;
		if (pstrWifiState->u8CurrState == M2M_WIFI_CONNECTED)
		{
			printf("wifi_cb: M2M_WIFI_CONNECTED\r\n");
			m2m_wifi_request_dhcp_client();
			OFF_M = false;
		}
		else if (pstrWifiState->u8CurrState == M2M_WIFI_DISCONNECTED)
		{
			pio_set_output(BUZZ_PIO, BUZZ_PIO_IDX_MASK, 0, 0, 0);

			printf("wifi_cb: M2M_WIFI_DISCONNECTED\r\n");
			gbConnectedWifi = false;
			wifi_connected = 0;
			//pwm buzzer aqui!!!!
			if (!OFF_M)
			{
				for (int i = 0; i < 20; i++)
				{
					pio_clear(BUZZ_PIO, BUZZ_PIO_IDX_MASK);
					delay_us(10000);
					pio_set(BUZZ_PIO, BUZZ_PIO_IDX_MASK);
					delay_us(10000);

				}
				pio_clear(BUZZ_PIO,BUZZ_PIO_IDX_MASK);
				OFF_M = true;
			}
		}

		break;
	}

	case M2M_WIFI_REQ_DHCP_CONF:
	{
		uint8_t *pu8IPAddress = (uint8_t *)pvMsg;
		printf("wifi_cb: IP address is %u.%u.%u.%u\r\n",
			   pu8IPAddress[0], pu8IPAddress[1], pu8IPAddress[2], pu8IPAddress[3]);
		wifi_connected = M2M_WIFI_CONNECTED;

		/* Obtain the IP Address by network name */
		//gethostbyname((uint8_t *)server_host_name);
		break;
	}

	default:
	{
		break;
	}
	}
}

/**
 * \brief This task, when activated, send every ten seconds on debug UART
 * the whole report of free heap and total tasks status
 */
static void task_monitor(void *pvParameters)
{
	static portCHAR szList[256];
	UNUSED(pvParameters);

	for (;;)
	{
		printf("--- task ## %u", (unsigned int)uxTaskGetNumberOfTasks());
		vTaskList((signed portCHAR *)szList);
		printf(szList);
		vTaskDelay(1000);
	}
}

static void task_wifi(void *pvParameters)
{
	vTaskDelay((TickType_t)3000 / portTICK_PERIOD_MS);
	/** TIME SEND/REC */
	tstrWifiInitParam param;
	int8_t ret;
	uint8_t mac_addr[6];
	uint8_t u8IsMacAddrValid;
	struct sockaddr_in addr_in;

	/*
       * O clock base do RTT é 32678Hz
       * Para gerar outra base de tempo é necessário
       * usar o PLL pre scale, que divide o clock base.
       *
       * Nesse exemplo, estamos operando com um clock base
       * de pllPreScale = 32768/32768/2 = 2Hz
       *
       * Quanto maior a frequência maior a resolução, porém
       * menor o tempo máximo que conseguimos contar.
       *
       * Podemos configurar uma IRQ para acontecer quando 
       * o contador do RTT atingir um determinado valor
       * aqui usamos o irqRTTvalue para isso.
       * 
       * Nesse exemplo o irqRTTvalue = 8, causando uma
       * interrupção a cada 2 segundos (lembre que usamos o 
       * pllPreScale, cada incremento do RTT leva 500ms (2Hz).
       */
	uint16_t pllPreScale = (int)(((float)32768) / 1000.0);
	uint32_t irqRTTvalue = 1;

	// reinicia RTT para gerar um novo IRQ
	RTT_init(pllPreScale, irqRTTvalue);

	/*
      * caso queira ler o valor atual do RTT, basta usar a funcao
      *   rtt_read_timer_value()
      */

	/* Initialize the BSP. */
	nm_bsp_init();

	/* Initialize Wi-Fi parameters structure. */
	memset((uint8_t *)&param, 0, sizeof(tstrWifiInitParam));

	/* Initialize Wi-Fi driver with data and status callbacks. */
	param.pfAppWifiCb = wifi_cb;
	ret = m2m_wifi_init(&param);
	if (M2M_SUCCESS != ret)
	{
		printf("main: m2m_wifi_init call error!(%d)\r\n", ret);
		while (1)
		{
		}
	}

	/* Initialize socket module. */
	socketInit();

	/* Register socket callback function. */
	registerSocketCallback(socket_cb, resolve_cb);

	/* Connect to router. */
	printf("main: connecting to WiFi AP %s...\r\n", (char *)MAIN_WLAN_SSID);
	m2m_wifi_connect((char *)MAIN_WLAN_SSID, sizeof(MAIN_WLAN_SSID), MAIN_WLAN_AUTH, (char *)MAIN_WLAN_PSK, M2M_WIFI_CH_ALL);

	addr_in.sin_family = AF_INET;
	addr_in.sin_port = _htons(MAIN_SERVER_PORT);
	inet_aton(MAIN_SERVER_NAME, &addr_in.sin_addr);
	printf("Inet aton : %d", addr_in.sin_addr);
	int timeout = 0;
	while (1)
	{

		m2m_wifi_handle_events(NULL);
		if (wifi_connected == M2M_WIFI_CONNECTED)
		{
			/* Open client socket. */
			if (tcp_client_socket < 0)
			{
				//printf("socket init \n");
				if ((tcp_client_socket = socket(AF_INET, SOCK_STREAM, 0)) < 0)
				{
					printf("main: failed to create TCP client socket error!\r\n");
					continue;
				}

				/* Connect server */
				//printf("socket connecting\n");

				if (connect(tcp_client_socket, (struct sockaddr *)&addr_in, sizeof(struct sockaddr_in)) != SOCK_ERR_NO_ERROR)
				{
					close(tcp_client_socket);
					tcp_client_socket = -1;
					printf("error\n");
				}
				else
				{
					gbTcpConnection = true;
				}
			}
		}
	}
}

/**
 * \brief Main application function.
 *
 * Initialize system, UART console, network then start weather client.
 *
 * \return Program return value.
 */
int main(void)
{

	/* Initialize the board. */
	sysclk_init();
	board_init();

	/* Initialize the UART console. */
	configure_console();
	printf(STRING_HEADER);

	if (xTaskCreate(task_wifi, "Wifi", TASK_WIFI_STACK_SIZE, NULL,
					TASK_WIFI_STACK_PRIORITY, NULL) != pdPASS)
	{
		printf("Failed to create Wifi task\r\n");
	}

	if (xTaskCreate(task_pwm, "pwm", TASK_BUZZER_STACK_SIZE, NULL,
					TASK_BUZZER_STACK_PRIORITY+1, NULL) != pdPASS)
	{
		printf("Failed to create pwm task\r\n");
	}

	vTaskStartScheduler();

	while (1)
	{
	};
	return 0;
}
