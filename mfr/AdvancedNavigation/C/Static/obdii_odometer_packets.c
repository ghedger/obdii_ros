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
 
 #include <stdint.h>
 #include <string.h>
 #include "an_packet_protocol.h"
 #include "obdii_odometer_packets.h"

/*
 * This file contains functions to decode and encode packets
 *
 * Decode functions take an an_packet_t and turn it into a type specific
 * to that packet so that the fields can be conveniently accessed. Decode
 * functions return 0 for success and 1 for failure. Decode functions are
 * used when receiving packets.
 *
 * Example decode
 *
 * an_packet_t an_packet
 * acknowledge_packet_t acknowledge_packet
 * ...
 * decode_acknowledge_packet(&acknowledge_packet, &an_packet);
 * printf("acknowledge id %d with result %d\n", acknowledge_packet.packet_id, acknowledge_packet.acknowledge_result);
 *
 * Encode functions take a type specific structure and turn it into an
 * an_packet_t. Encode functions are used when sending packets.
 *
 * Example encode
 *
 * an_packet_t an_packet;
 * boot_mode_packet_t boot_mode_packet;
 * ...
 * boot_mode_packet.boot_mode = boot_mode_bootloader;
 * encode_boot_mode_packet(&an_packet, &boot_mode_packet);
 * serial_port_transmit(an_packet_pointer(&an_packet), an_packet_size(&an_packet));
 *
 */

int decode_acknowledge_packet(acknowledge_packet_t *acknowledge_packet, an_packet_t *an_packet)
{
	if(an_packet->id == packet_id_acknowledge && an_packet->length == 4)
	{
		acknowledge_packet->packet_id = an_packet->data[0];
		memcpy(&acknowledge_packet->packet_crc, &an_packet->data[1], sizeof(uint16_t));
		acknowledge_packet->acknowledge_result = an_packet->data[3];
		return 0;
	}
	else return 1;
}

void encode_request_packet(an_packet_t *an_packet, uint8_t requested_packet_id)
{
	an_packet->id = packet_id_request;
	an_packet->length = 1;
	an_packet->data[0] = requested_packet_id;
}

int decode_boot_mode_packet(boot_mode_packet_t *boot_mode_packet, an_packet_t *an_packet)
{
	if(an_packet->id == packet_id_boot_mode && an_packet->length == 1)
	{
		boot_mode_packet->boot_mode = an_packet->data[0];
		return 0;
	}
	else return 1;
}

void encode_boot_mode_packet(an_packet_t *an_packet, boot_mode_packet_t *boot_mode_packet)
{
	an_packet->id = packet_id_boot_mode;
	an_packet->length = 1;
	an_packet->data[0] = boot_mode_packet->boot_mode;
}

int decode_device_information_packet(device_information_packet_t *device_information_packet, an_packet_t *an_packet)
{
	if(an_packet->id == packet_id_device_information && an_packet->length == 24)
	{
		memcpy(&device_information_packet->software_version, &an_packet->data[0], sizeof(uint32_t));
		memcpy(&device_information_packet->device_id, &an_packet->data[4], sizeof(uint32_t));
		memcpy(&device_information_packet->hardware_revision, &an_packet->data[8], sizeof(uint32_t));
		memcpy(&device_information_packet->serial_number[0], &an_packet->data[12], 3*sizeof(uint32_t));
		return 0;
	}
	else return 1;
}

void encode_restore_factory_settings_packet(an_packet_t *an_packet)
{
	uint32_t verification = 0x85429E1C;
	an_packet->id = packet_id_restore_factory_settings;
	an_packet->length = 4;
	memcpy(&an_packet->data[0], &verification, sizeof(uint32_t));
}

void encode_reset_packet(an_packet_t *an_packet)
{
	uint32_t verification = 0x21057A7E;
	an_packet->id = packet_id_reset;
	an_packet->length = 4;
	memcpy(&an_packet->data[0], &verification, sizeof(uint32_t));
}

/* State packets */

int decode_odometer_packet(odometer_packet_t *odometer_packet, an_packet_t *an_packet)
{
	if(an_packet->id == packet_id_odometer && an_packet->length == 13)
	{
		memcpy(&odometer_packet->delay, &an_packet->data[0], sizeof(float));
		memcpy(&odometer_packet->speed, &an_packet->data[4], sizeof(float));
		memcpy(&odometer_packet->distance_travelled, &an_packet->data[8], sizeof(float));
		odometer_packet->flags.r = an_packet->data[12];
		return 0;
	}
	else return 1;
}
