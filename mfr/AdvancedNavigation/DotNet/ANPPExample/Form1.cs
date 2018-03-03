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
using System.ComponentModel;
using System.Data;
using System.Drawing;
using System.Linq;
using System.Text;
using System.Threading;
using System.Threading.Tasks;
using System.Windows.Forms;

namespace ANPPExample
{
    public partial class Form1 : Form
    {
        Boolean serialConnected = false;
        ANPacketDecoder anPacketDecoder;

        public Form1()
        {
            InitializeComponent();
            anPacketDecoder = new ANPacketDecoder();
        }

        private void Form1_Load(object sender, EventArgs e)
        {
            comboBaud.SelectedIndex = 7;
            updatePorts();
        }

        private void timer1_Tick(object sender, EventArgs e)
        {
            updatePorts();
        }

        private void updatePorts()
        {
            if (!serialConnected)
            {
                string[] ports = System.IO.Ports.SerialPort.GetPortNames();
                Boolean portsChanged = ports.Length != comboPort.Items.Count;
                if (!portsChanged)
                {
                    for(int i=0; i < ports.Length; i++)
                    {
                        if(ports[i].CompareTo(comboPort.Items[i].ToString()) != 0)
                        {
                            portsChanged = true;
                            break;
                        }
                    }
                }
                if (portsChanged)
                {
                    comboPort.Items.Clear();
                    foreach (string portName in ports)
                    {
                        comboPort.Items.Add((object)portName);
                    }
                    if (ports.Length > 0)
                    {
                        comboPort.SelectedIndex = 0;
                    }
                }
            }
        }

        private void buttonConnect_Click(object sender, EventArgs e)
        {
            if (serialConnected)
            {
                try
                {
                    serialPort1.Close();
                    buttonConnect.Text = "Connect";
                    serialConnected = false;
                }
                catch { }
            }
            else
            {
                try
                {
                    serialPort1.PortName = comboPort.Text;
                    serialPort1.BaudRate = Convert.ToInt32(comboBaud.Text);
                    serialPort1.Open();
                    buttonConnect.Text = "Disconnect";
                    serialConnected = true;
                }
                catch { }
            }
        }

        private void serialPort1_DataReceived(object sender, System.IO.Ports.SerialDataReceivedEventArgs e)
        {
            try
            {
                anPacketDecoder.bufferLength += serialPort1.Read(anPacketDecoder.buffer, anPacketDecoder.bufferLength, anPacketDecoder.buffer.Length - anPacketDecoder.bufferLength);
                ANPacket packet = null;
                while ((packet = anPacketDecoder.packetDecode()) != null)
                {
                    switch (packet.id)
                    {
                        case ANPacket.PACKET_ID_ODOMETER:
                            if (packet.length == 13)
                            {
                                ANPacket67 anPacket67 = new ANPacket67(packet);
                                this.Invoke((MethodInvoker)delegate
                                {
                                    richTextBox1.AppendText("Received Odometer Packet\n");
                                    textBoxDelay.Text = anPacket67.delay.ToString();
                                    textBoxSpeed.Text = anPacket67.speed.ToString();
                                    textBoxDistance.Text = anPacket67.distanceTravelled.ToString();
                                    checkBoxReverseDetection.Checked = anPacket67.reverseDetectionSupported;
                                });
                            }
                            break;
                    }
                }
            }
            catch { }
        }
    }
}
