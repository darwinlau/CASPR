/*******************************************************************************
* Copyright (c) 2016, ROBOTIS CO., LTD.
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
* * Redistributions of source code must retain the above copyright notice, this
*   list of conditions and the following disclaimer.
*
* * Redistributions in binary form must reproduce the above copyright notice,
*   this list of conditions and the following disclaimer in the documentation
*   and/or other materials provided with the distribution.
*
* * Neither the name of ROBOTIS nor the names of its
*   contributors may be used to endorse or promote products derived from
*   this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
* FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
* DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
* OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
* OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*******************************************************************************/

/* Author: Ryu Woon Jung (Leon) */

#if defined(_WIN32) || defined(_WIN64)
#define WINDLLEXPORT
#endif

#include <stdio.h>
#include <string.h>
#include <time.h>
#include "dynamixel_sdk_windows/port_handler_windows.h"

#define LATENCY_TIMER  16 // msec (USB latency timer)

typedef struct
{
  HANDLE  serial_handle;
  LARGE_INTEGER freq, counter;

  int     baudrate;
  char    port_name[30];

  double  packet_start_time;
  double  packet_timeout;
  double  tx_time_per_byte;
}PortData;

static PortData *portData;

int portHandlerWindows(const char *port_name)
{
  int port_num;
  char buffer[15];

  sprintf_s(buffer, sizeof(buffer), "\\\\.\\%s", port_name);

  if (portData == NULL)
  {
    port_num = 0;
    g_used_port_num = 1;
    portData = (PortData*)calloc(1, sizeof(PortData));
    g_is_using = (uint8_t*)calloc(1, sizeof(uint8_t));
  }
  else
  {
    for (port_num = 0; port_num < g_used_port_num; port_num++)
    {
      if (!strcmp(portData[port_num].port_name, buffer))
        break;
    }

    if (port_num == g_used_port_num)
    {
      for (port_num = 0; port_num < g_used_port_num; port_num++)
      {
        if (portData[port_num].serial_handle != INVALID_HANDLE_VALUE)
          break;
      }

      if (port_num == g_used_port_num)
      {
        g_used_port_num++;
        portData = (PortData*)realloc(portData, g_used_port_num * sizeof(PortData));
        g_is_using = (uint8_t*)realloc(g_is_using, g_used_port_num * sizeof(uint8_t));
      }
    }
    else
    {
      printf("[PortHandler setup] The port number %d has same device name... reinitialize port number %d!!\n", port_num, port_num);
    }
  }

  portData[port_num].serial_handle = INVALID_HANDLE_VALUE;
  portData[port_num].baudrate = DEFAULT_BAUDRATE;
  portData[port_num].packet_start_time = 0.0;
  portData[port_num].packet_timeout = 0.0;
  portData[port_num].tx_time_per_byte = 0.0;

  g_is_using[port_num] = False;

  setPortNameWindows(port_num, buffer);

  return port_num;
}

uint8_t openPortWindows(int port_num)
{
  return setBaudRateWindows(port_num, portData[port_num].baudrate);
}

void closePortWindows(int port_num)
{
  if (portData[port_num].serial_handle != INVALID_HANDLE_VALUE)
  {
    CloseHandle(portData[port_num].serial_handle);
    portData[port_num].serial_handle = INVALID_HANDLE_VALUE;
  }
}

void clearPortWindows(int port_num)
{
  PurgeComm(portData[port_num].serial_handle, PURGE_RXABORT | PURGE_RXCLEAR);
}

void setPortNameWindows(int port_num, const char *port_name)
{
  strcpy_s(portData[port_num].port_name, sizeof(portData[port_num].port_name), port_name);
}

char *getPortNameWindows(int port_num)
{
  return portData[port_num].port_name;
}

uint8_t setBaudRateWindows(int port_num, const int baudrate)
{
  closePortWindows(port_num);

  portData[port_num].baudrate = baudrate;
  return setupPortWindows(port_num, baudrate);
}

int getBaudRateWindows(int port_num)
{
  return portData[port_num].baudrate;
}

int readPortWindows(int port_num, uint8_t *packet, int length)
{
  DWORD dwRead = 0;

  if (ReadFile(portData[port_num].serial_handle, packet, (DWORD)length, &dwRead, NULL) == FALSE)
    return -1;

  return (int)dwRead;
}

int writePortWindows(int port_num, uint8_t *packet, int length)
{
  DWORD dwWrite = 0;

  if (WriteFile(portData[port_num].serial_handle, packet, (DWORD)length, &dwWrite, NULL) == FALSE)
    return -1;

  return (int)dwWrite;
}

void setPacketTimeoutWindows(int port_num, uint16_t packet_length)
{
  portData[port_num].packet_start_time = getCurrentTimeWindows(port_num);
  portData[port_num].packet_timeout = (portData[port_num].tx_time_per_byte * (double)packet_length) + (LATENCY_TIMER * 2.0) + 2.0;
}

void setPacketTimeoutMSecWindows(int port_num, double msec)
{
  portData[port_num].packet_start_time = getCurrentTimeWindows(port_num);
  portData[port_num].packet_timeout = msec;
}

uint8_t isPacketTimeoutWindows(int port_num)
{
  if (getTimeSinceStartWindows(port_num) > portData[port_num].packet_timeout)
  {
    portData[port_num].packet_timeout = 0;
    return True;
  }
  return False;
}

double getCurrentTimeWindows(int port_num)
{
  QueryPerformanceCounter(&portData[port_num].counter);
  QueryPerformanceFrequency(&portData[port_num].freq);
  return (double)portData[port_num].counter.QuadPart / (double)portData[port_num].freq.QuadPart * 1000.0;
}

double getTimeSinceStartWindows(int port_num)
{
  double time_since_start;

  time_since_start = getCurrentTimeWindows(port_num) - portData[port_num].packet_start_time;
  if (time_since_start < 0.0)
    portData[port_num].packet_start_time = getCurrentTimeWindows(port_num);

  return time_since_start;
}

uint8_t setupPortWindows(int port_num, const int baudrate)
{
  DCB dcb;
  COMMTIMEOUTS timeouts;
  DWORD dwError;

  closePortWindows(port_num);

  portData[port_num].serial_handle = CreateFileA(portData[port_num].port_name, GENERIC_READ | GENERIC_WRITE, 0, NULL, OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, NULL);
  if (portData[port_num].serial_handle == INVALID_HANDLE_VALUE)
  {
    printf("[PortHandlerWindows::SetupPort] Error opening serial port!\n");
    return False;
  }

  dcb.DCBlength = sizeof(DCB);
  if (GetCommState(portData[port_num].serial_handle, &dcb) == FALSE)
    goto DXL_HAL_OPEN_ERROR;

  // Set baudrate
  dcb.BaudRate = (DWORD)baudrate;
  dcb.ByteSize = 8;                    // Data bit = 8bit
  dcb.Parity = NOPARITY;             // No parity
  dcb.StopBits = ONESTOPBIT;           // Stop bit = 1
  dcb.fParity = NOPARITY;             // No Parity check
  dcb.fBinary = 1;                    // Binary mode
  dcb.fNull = 0;                    // Get Null byte
  dcb.fAbortOnError = 0;
  dcb.fErrorChar = 0;
  // Not using XOn/XOff
  dcb.fOutX = 0;
  dcb.fInX = 0;
  // Not using H/W flow control
  dcb.fDtrControl = DTR_CONTROL_DISABLE;
  dcb.fRtsControl = RTS_CONTROL_DISABLE;
  dcb.fDsrSensitivity = 0;
  dcb.fOutxDsrFlow = 0;
  dcb.fOutxCtsFlow = 0;

  if (SetCommState(portData[port_num].serial_handle, &dcb) == FALSE)
    goto DXL_HAL_OPEN_ERROR;

  if (SetCommMask(portData[port_num].serial_handle, 0) == FALSE) // Not using Comm event
    goto DXL_HAL_OPEN_ERROR;
  if (SetupComm(portData[port_num].serial_handle, 4096, 4096) == FALSE) // Buffer size (Rx,Tx)
    goto DXL_HAL_OPEN_ERROR;
  if (PurgeComm(portData[port_num].serial_handle, PURGE_TXABORT | PURGE_TXCLEAR | PURGE_RXABORT | PURGE_RXCLEAR) == FALSE) // Clear buffer
    goto DXL_HAL_OPEN_ERROR;
  if (ClearCommError(portData[port_num].serial_handle, &dwError, NULL) == FALSE)
    goto DXL_HAL_OPEN_ERROR;

  if (GetCommTimeouts(portData[port_num].serial_handle, &timeouts) == FALSE)
    goto DXL_HAL_OPEN_ERROR;
  // Timeout (Not using timeout)
  // Immediatly return
  timeouts.ReadIntervalTimeout = 0;
  timeouts.ReadTotalTimeoutMultiplier = 0;
  timeouts.ReadTotalTimeoutConstant = 1; // must not be zero.
  timeouts.WriteTotalTimeoutMultiplier = 0;
  timeouts.WriteTotalTimeoutConstant = 0;
  if (SetCommTimeouts(portData[port_num].serial_handle, &timeouts) == FALSE)
    goto DXL_HAL_OPEN_ERROR;

  portData[port_num].tx_time_per_byte = (1000.0 / (double)portData[port_num].baudrate) * 10.0;
  return True;

  DXL_HAL_OPEN_ERROR:
    closePortWindows(port_num);

  return False;
}
