/****************************************************************/
/*                                                              */
/*          Advanced Navigation Packet Protocol Library         */
/*       C Language Static OBDII Odometer SDK, Version 1.0      */
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

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <math.h>
#include "an_packet_protocol.h"
#include "obdii_odometer_packets.h"
#include "obdii_odometer_data_dump.h"

void an_packet_transmit(an_packet_t *an_packet)
{
	an_packet_encode(an_packet);

	/* it is up to the user to define the data transmit function which should have
	 * the prototype: send_function(uint8_t *data, int length); example usage below
	 *
	 * serial_port_transmit(an_packet_pointer(an_packet), an_packet_size(an_packet));
	 */
}

int main()
{
	an_decoder_t an_decoder;
	an_packet_t an_packet;
	int data_count = 0;
	int bytes_to_copy;
	
	odometer_packet_t odometer_packet;
	
	an_decoder_initialise(&an_decoder);
	
	/* iterate through an example data dump from OBDII Odometer */
	while(data_count < sizeof(obdii_odometer_data_dump))
	{
		/* calculate the number of bytes to copy */
		bytes_to_copy = an_decoder_size(&an_decoder);
		if(bytes_to_copy > sizeof(obdii_odometer_data_dump) - data_count) bytes_to_copy = sizeof(obdii_odometer_data_dump) - data_count;
		
		/* fill the decode buffer with the data */
		memcpy(an_decoder_pointer(&an_decoder), &obdii_odometer_data_dump[data_count], bytes_to_copy*sizeof(uint8_t));
		an_decoder_increment(&an_decoder, bytes_to_copy);
		
		/* increase the iterator by the number of bytes copied */
		data_count += bytes_to_copy;
		
		/* decode all packets in the internal buffer */
		while(an_packet_decode(&an_decoder, &an_packet))
		{
			if(an_packet.id == packet_id_odometer) /* odometer packet */
			{
				/* copy all the binary data into the typedef struct for the packet */
				/* this allows easy access to all the different values             */
				if(decode_odometer_packet(&odometer_packet, &an_packet) == 0)
				{
					printf("Odometer Packet:\n");
					printf("\tDelay = %f s, Speed = %f m/s, Distance = %f m, Reverse Detection = %d\n", odometer_packet.delay, odometer_packet.speed, odometer_packet.distance_travelled, odometer_packet.flags.b.reverse_detection_supported);
				}
			}
			else
			{		
				printf("Packet ID %u of Length %u\n", an_packet.id, an_packet.length);
			}
		}
	}
	
	return EXIT_SUCCESS;
}
