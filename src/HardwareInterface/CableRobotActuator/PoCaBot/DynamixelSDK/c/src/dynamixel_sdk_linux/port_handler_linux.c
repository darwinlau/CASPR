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

#include <stdio.h>
#include <fcntl.h>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <termios.h>
#include <time.h>
#include <sys/time.h>
#include <sys/ioctl.h>
#include <linux/serial.h>

#include "dynamixel_sdk_linux/port_handler_linux.h"

#define LATENCY_TIMER   4  // msec (USB latency timer)

typedef struct
{
  int     socket_fd;
  int     baudrate;
  char    port_name[30];

  double  packet_start_time;
  double  packet_timeout;
  double  tx_time_per_byte;
}PortData;

static PortData *portData;

int portHandlerLinux(const char *port_name)
{
  int port_num;

  if (portData == NULL)
  {
    port_num = 0;
    g_used_port_num = 1;
    portData = (PortData *)calloc(1, sizeof(PortData));
    g_is_using = (uint8_t*)calloc(1, sizeof(uint8_t));
  }
  else
  {
    for (port_num = 0; port_num < g_used_port_num; port_num++)
    {
      if (!strcmp(portData[port_num].port_name, port_name))
        break;
    }

    if (port_num == g_used_port_num)
    {
      for (port_num = 0; port_num < g_used_port_num; port_num++)
      {
        if (portData[port_num].socket_fd != -1)
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

  portData[port_num].socket_fd = -1;
  portData[port_num].baudrate = DEFAULT_BAUDRATE;
  portData[port_num].packet_start_time = 0.0;
  portData[port_num].packet_timeout = 0.0;
  portData[port_num].tx_time_per_byte = 0.0;

  g_is_using[port_num] = False;

  setPortNameLinux(port_num, port_name);

  return port_num;
}

uint8_t openPortLinux(int port_num)
{
  return setBaudRateLinux(port_num, portData[port_num].baudrate);
}

void closePortLinux(int port_num)
{
  if (portData[port_num].socket_fd != -1)
  {
    close(portData[port_num].socket_fd);
    portData[port_num].socket_fd = -1;
  }
}

void clearPortLinux(int port_num)
{
  tcflush(portData[port_num].socket_fd, TCIOFLUSH);
}

void setPortNameLinux(int port_num, const char *port_name)
{
  strcpy(portData[port_num].port_name, port_name);
}

char *getPortNameLinux(int port_num)
{
  return portData[port_num].port_name;
}

uint8_t setBaudRateLinux(int port_num, const int baudrate)
{
  int baud = getCFlagBaud(baudrate);

  closePortLinux(port_num);

  if (baud <= 0)   // custom baudrate
  {
    setupPortLinux(port_num, B38400);
    portData[port_num].baudrate = baudrate;
    return setCustomBaudrateLinux(port_num, baudrate);
  }
  else
  {
    portData[port_num].baudrate = baudrate;
    return setupPortLinux(port_num, baud);
  }
}

int getBaudRateLinux(int port_num)
{
  return portData[port_num].baudrate;
}

int getBytesAvailableLinux(int port_num)
{
  int bytes_available;
  ioctl(portData[port_num].socket_fd, FIONREAD, &bytes_available);
  return bytes_available;
}

int readPortLinux(int port_num, uint8_t *packet, int length)
{
  return read(portData[port_num].socket_fd, packet, length);
}

int writePortLinux(int port_num, uint8_t *packet, int length)
{
  return write(portData[port_num].socket_fd, packet, length);
}

void setPacketTimeoutLinux(int port_num, uint16_t packet_length)
{
  portData[port_num].packet_start_time = getCurrentTimeLinux();
  portData[port_num].packet_timeout = (portData[port_num].tx_time_per_byte * (double)packet_length) + (LATENCY_TIMER * 2.0) + 2.0;
}

void setPacketTimeoutMSecLinux(int port_num, double msec)
{
  portData[port_num].packet_start_time = getCurrentTimeLinux();
  portData[port_num].packet_timeout = msec;
}

uint8_t isPacketTimeoutLinux(int port_num)
{
  if (getTimeSinceStartLinux(port_num) > portData[port_num].packet_timeout)
  {
    portData[port_num].packet_timeout = 0;
    return True;
  }
  return False;
}

double getCurrentTimeLinux()
{
  struct timespec tv;
  clock_gettime(CLOCK_REALTIME, &tv);
  return ((double)tv.tv_sec*1000.0 + (double)tv.tv_nsec*0.001*0.001);
}

double getTimeSinceStartLinux(int port_num)
{
  double time_since_start;

  time_since_start = getCurrentTimeLinux() - portData[port_num].packet_start_time;
  if (time_since_start < 0.0)
    portData[port_num].packet_start_time = getCurrentTimeLinux();

  return time_since_start;
}

uint8_t setupPortLinux(int port_num, int cflag_baud)
{
  struct termios newtio;

  portData[port_num].socket_fd = open(portData[port_num].port_name, O_RDWR | O_NOCTTY | O_NONBLOCK);

  if (portData[port_num].socket_fd < 0)
  {
    printf("[PortHandlerLinux::SetupPort] Error opening serial port!\n");
    return False;
  }

  bzero(&newtio, sizeof(newtio)); // clear struct for new port settings

  newtio.c_cflag = cflag_baud | CS8 | CLOCAL | CREAD;
  newtio.c_iflag = IGNPAR;
  newtio.c_oflag = 0;
  newtio.c_lflag = 0;
  newtio.c_cc[VTIME] = 0;
  newtio.c_cc[VMIN] = 0;

  // clean the buffer and activate the settings for the port
  tcflush(portData[port_num].socket_fd, TCIFLUSH);
  tcsetattr(portData[port_num].socket_fd, TCSANOW, &newtio);

  portData[port_num].tx_time_per_byte = (1000.0 / (double)portData[port_num].baudrate) * 10.0;
  return True;
}

uint8_t setCustomBaudrateLinux(int port_num, int speed)
{
  // try to set a custom divisor
  struct serial_struct ss;
  if (ioctl(portData[port_num].socket_fd, TIOCGSERIAL, &ss) != 0)
  {
    printf("[PortHandlerLinux::SetCustomBaudrate] TIOCGSERIAL failed!\n");
    return False;
  }

  ss.flags = (ss.flags & ~ASYNC_SPD_MASK) | ASYNC_SPD_CUST;
  ss.custom_divisor = (ss.baud_base + (speed / 2)) / speed;
  int closest_speed = ss.baud_base / ss.custom_divisor;

  if (closest_speed < speed * 98 / 100 || closest_speed > speed * 102 / 100)
  {
    printf("[PortHandlerLinux::setCustomBaudrate] Cannot set speed to %d, closest is %d \n", speed, closest_speed);
    return False;
  }

  if (ioctl(portData[port_num].socket_fd, TIOCSSERIAL, &ss) < 0)
  {
    printf("[PortHandlerLinux::setCustomBaudrate] TIOCSSERIAL failed!\n");
    return False;
  }

  portData[port_num].tx_time_per_byte = (1000.0 / (double)speed) * 10.0;
  return True;
}

int getCFlagBaud(int baudrate)
{
  switch (baudrate)
  {
    case 9600:
      return B9600;
    case 19200:
      return B19200;
    case 38400:
      return B38400;
    case 57600:
      return B57600;
    case 115200:
      return B115200;
    case 230400:
      return B230400;
    case 460800:
      return B460800;
    case 500000:
      return B500000;
    case 576000:
      return B576000;
    case 921600:
      return B921600;
    case 1000000:
      return B1000000;
    case 1152000:
      return B1152000;
    case 1500000:
      return B1500000;
    case 2000000:
      return B2000000;
    case 2500000:
      return B2500000;
    case 3000000:
      return B3000000;
    case 3500000:
      return B3500000;
    case 4000000:
      return B4000000;
    default:
      return -1;
  }
}
