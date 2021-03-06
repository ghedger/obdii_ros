/****************************************************************/
/*                                                              */
/*          Advanced Navigation Packet Protocol Library         */
/*       .NET C# Language OBDII Odometer SDK, Version 1.0       */
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

using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace ANPPExample
{
    class ANPacket67
    {
        public float delay;
        public float speed;
        public float distanceTravelled;
        public Boolean reverseDetectionSupported;

        public ANPacket67()
        {
            delay = 0;
            speed = 0;
            distanceTravelled = 0;
            reverseDetectionSupported = false;
        }

        public ANPacket67(ANPacket packet)
        {
            delay = BitConverter.ToSingle(packet.data, 0);
            speed = BitConverter.ToSingle(packet.data, 4);
            distanceTravelled = BitConverter.ToSingle(packet.data, 8);
            reverseDetectionSupported = (packet.data[0] & 0x01) != 0;
        }
    }
}
