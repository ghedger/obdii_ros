/****************************************************************/
/*                                                              */
/*          Advanced Navigation Packet Protocol Library         */
/*      C Language Dynamic OBDII Odometer SDK, Version 1.0      */
/*   Copyright 2014, Xavier Orr, Advanced Navigation Pty Ltd    */
/*                                                              */
/****************************************************************/
/*
 * Copyright (C) 2014 Advanced Navigation Pty Ltd
 *
 * Permission is hereby granted, free of charge, to any person obtaining
 * a copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included
 * in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS
 * OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE 
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 */

#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <math.h>

#ifdef _WIN32
#include <Windows.h>
#else
#include <unistd.h>
#endif

#include "rs232/rs232.h"
#include "an_packet_protocol.h"
#include "obdii_odometer_packets.h"

#include "odom.h"

/* GPH MODIFIED 20180302
*/
#if !defined(DEBUG)
#define DEBUG 1
#endif
#if defined(DEBUG)
#define ODOM_LOG printf
#else
#define ODOM_LOG
#endif
#define ODOM_ERROR printf

/* END GPH
*/
int an_packet_transmit(an_packet_t *an_packet)
{
	an_packet_encode(an_packet);
	return SendBuf(an_packet_pointer(an_packet), an_packet_size(an_packet));
}

/*
 * This is an example of sending a packet to OBDII Odometer.
 *
 * 1. First declare the structure for the packet, in this case request_packet_t.
 * 2. Set all the fields of the packet structure
 * 3. Encode the packet structure into an an_packet_t using the appropriate helper function
 * 4. Send the packet
 * 5. Free the packet
 */
void request_device_information()
{
	an_packet_t *an_packet = an_packet_allocate(1, packet_id_request);
	an_packet->data[0] = packet_id_device_information;
	an_packet_transmit(an_packet);
	an_packet_free(&an_packet);
}

void *odom_thread(void *pv)
{
	an_decoder_t an_decoder;
	an_packet_t *an_packet;
	odometer_packet_t odometer_packet;

	int bytes_received;

  const WorkerParams *pWorkerParams = ( const WorkerParams * ) pv;

  ODOM_LOG("ODOM WORKER THREAD STARTING\n");

	/* open the com port */
  char szPort[MAX_PORT_STR_SIZE];
  strncpy(szPort, pWorkerParams->port_.c_str(), sizeof(szPort));

  ODOM_LOG(
    "ODOM WORKER THREAD: Attempting to open port %s at %d baud.\n",
    szPort,
    pWorkerParams->baud_rate_
  );

	if (OpenComport( szPort, pWorkerParams->baud_rate_ ))
	{
		ODOM_ERROR("Could not open serial port %s at %d baud.\n",
      szPort,
      pWorkerParams->baud_rate_
    );
    ODOM_ERROR("ODOM WORKER EXITING...\n");
    return NULL;
		//exit(EXIT_FAILURE);
	}

	an_decoder_initialise(&an_decoder);

	while (1)
	{
		if ((bytes_received = PollComport(an_decoder_pointer(&an_decoder), an_decoder_size(&an_decoder))) > 0)
		{
			/* increment the decode buffer length by the number of bytes received */
			an_decoder_increment(&an_decoder, bytes_received);

			/* decode all the packets in the buffer */
			while ((an_packet = an_packet_decode(&an_decoder)) != NULL)
			{
				if (an_packet->id == packet_id_odometer) /* odometer packet */
				{
					/* copy all the binary data into the typedef struct for the packet */
					/* this allows easy access to all the different values             */
					if(decode_odometer_packet(&odometer_packet, an_packet) == 0)
					{
						ODOM_LOG("Odometer Packet:\n");
						ODOM_LOG("\tDelay = %f s, Speed = %f m/s, Distance = %f m, Reverse Detection = %d\n", odometer_packet.delay, odometer_packet.speed, odometer_packet.distance_travelled, odometer_packet.flags.b.reverse_detection_supported);
					}
				}
				else
				{
					ODOM_LOG("Packet ID %u of Length %u\n", an_packet->id, an_packet->length);
				}
				/* Ensure that you free the an_packet when your done with it or you will leak memory */
				an_packet_free(&an_packet);
			}
		}
#ifdef _WIN32
    Sleep(10);
#else
    usleep(10000);
#endif
	}
  return NULL;
}
