/****************************************************************/
/*                                                              */
/*      Advanced Navigation Packet Protocol Library             */
/*            Java Language SDK, Version 1.0                    */
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

package com.advancednavigation.anPackets;

public class ANPacket
{
	public static final int PACKET_ID_ACKNOWLEDGE = 0;
	public static final int PACKET_ID_REQUEST = 1;
	public static final int PACKET_ID_BOOT_MODE = 2;
	public static final int PACKET_ID_DEVICE_INFORMATION = 3;
	public static final int PACKET_ID_RESTORE_FACTORY_SETTINGS = 4;
	public static final int PACKET_ID_RESET = 5;
	public static final int PACKET_ID_PRINT = 6;
	public static final int PACKET_ID_FILE_TRANSFER_REQUEST = 7;
	public static final int PACKET_ID_FILE_TRANSFER_ACKNOWLEDGE = 8;
	public static final int PACKET_ID_FILE_TRANSFER = 9;
	
	public static final int PACKET_ID_EXTERNAL_ODOMETER = 67;
	
	public int id;
	public int length;
	public byte[] data;
	
	public ANPacket(int length, int id)
	{
		this.length = length;
		this.id = id;
		this.data = new byte[length];
	}
}
