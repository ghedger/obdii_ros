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

package com.advancednavigation.obdiiOdometerViewer;

import jssc.*;

import java.awt.EventQueue;

import javax.imageio.ImageIO;
import javax.swing.ImageIcon;
import javax.swing.JFrame;
import javax.swing.JScrollPane;
import javax.swing.SwingUtilities;
import javax.swing.UIManager;
import javax.swing.JComboBox;
import javax.swing.JButton;
import javax.swing.JTextArea;
import javax.swing.DefaultComboBoxModel;
import com.advancednavigation.anPackets.*;

import java.awt.GridBagLayout;
import java.awt.GridBagConstraints;
import java.awt.Insets;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.io.*;
import java.util.Date;
import java.util.GregorianCalendar;

import javax.swing.JPanel;
import javax.swing.border.TitledBorder;
import javax.swing.JLabel;
import javax.swing.JTextField;
import javax.swing.SwingConstants;
import javax.swing.JMenuBar;
import javax.swing.JMenu;
import javax.swing.JMenuItem;
import javax.swing.JCheckBox;

public class Main
{
	private JFrame frmOBDIIViewer;
	private JFrame frmAbout = null;
	private JComboBox<String> comboBoxPort;
	private JComboBox<String> comboBoxBaud;
	private JButton btnConnect;

	// Serial Port
	private SerialPort serialPort;
	protected Boolean serialConnected = false;
	private Boolean serialPortsAvailable = false;
	private JTextArea textArea;
	private JPanel panel;
	private JLabel lblSpeed;
	private JLabel lblRate;
	protected JTextField textFieldSpeed;
	protected JTextField textFieldRate;
	private JLabel lblKmh;
	private JLabel lblHz;
	int packetCount = 0;
	private JMenuBar menuBar;
	private JMenu mnFile;
	private JMenu mnHelp;
	private JMenuItem mntmAbout;
	private JMenuItem mntmExit;
	private JLabel lblDelay;
	private JTextField textFieldDelay;
	private JLabel lblSec;
	private JCheckBox chckbxReversingDetection;
	private JLabel lblDistance;
	private JTextField textFieldDistance;
	private JLabel lblMetres;

	/*
	 * Code for sending packets
	 * 
	 * ANPacket packet = new ANPacket(1, ANPacket.PACKET_ID_REQUEST);
	 * packet.data[0] = ANPacket.PACKET_ID_BOOT_MODE;
	 * serialOutput.write(ANPacketDecoder.packetEncode(packet));
	 * 
	 * The above example sends a packet of length 1 requesting the device boot
	 * mode packet The serial port must be connected for the above code to work,
	 * you can use the boolean serialConnected to check this
	 */

	/**
	 * Launch the application.
	 */
	public static void main(String[] args)
	{
		EventQueue.invokeLater(new Runnable()
		{
			public void run()
			{
				try
				{
					Main window = new Main();
					window.frmOBDIIViewer.setVisible(true);
				}
				catch (Exception e)
				{
					e.printStackTrace();
				}
			}
		});
	}

	/**
	 * Create the application.
	 */
	public Main()
	{
		initialize();
	}

	/**
	 * Initialize the contents of the frame.
	 */
	private void initialize()
	{
		// this sets swing to use the native look and feel, much more
		// aesthetically pleasing
		try
		{
			UIManager.setLookAndFeel(UIManager.getSystemLookAndFeelClassName());
		}
		catch (Exception e)
		{
		}

		// all of the swing components and layout management is best edited
		// using the windowbuilder GUI
		frmOBDIIViewer = new JFrame();
		frmOBDIIViewer.setTitle("OBDII Odometer Viewer");
		frmOBDIIViewer.setBounds(100, 100, 523, 416);
		frmOBDIIViewer.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
		GridBagLayout gridBagLayout = new GridBagLayout();
		gridBagLayout.columnWidths = new int[] { 10, 0, 0, 90, 5, 0 };
		gridBagLayout.rowHeights = new int[] { 10, 0, 0, 0, 5, 0 };
		gridBagLayout.columnWeights = new double[] { 0.0, 1.0, 1.0, 0.0, 0.0, Double.MIN_VALUE };
		gridBagLayout.rowWeights = new double[] { 0.0, 0.0, 0.0, 1.0, 0.0, Double.MIN_VALUE };
		frmOBDIIViewer.getContentPane().setLayout(gridBagLayout);
		try
		{
			frmOBDIIViewer.setIconImage(ImageIO.read(Main.class.getResourceAsStream("/com/advancednavigation/images/an_icon.png")));
		}
		catch (IOException e)
		{
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		

		comboBoxPort = new JComboBox<String>();
		comboBoxPort.setEnabled(false);
		comboBoxPort.setModel(new DefaultComboBoxModel<String>(new String[] { "No Serial Ports" }));
		GridBagConstraints gbc_comboBoxPort = new GridBagConstraints();
		gbc_comboBoxPort.insets = new Insets(0, 0, 5, 5);
		gbc_comboBoxPort.fill = GridBagConstraints.HORIZONTAL;
		gbc_comboBoxPort.gridx = 1;
		gbc_comboBoxPort.gridy = 1;
		frmOBDIIViewer.getContentPane().add(comboBoxPort, gbc_comboBoxPort);

		comboBoxBaud = new JComboBox<String>();
		comboBoxBaud.setModel(new DefaultComboBoxModel<String>(new String[] { "1200", "2400", "4800", "9600", "19200", "38400", "57600", "115200", "230400", "460800", "500000", "921600", "1000000" }));
		comboBoxBaud.setSelectedIndex(7);
		GridBagConstraints gbc_comboBoxBaud = new GridBagConstraints();
		gbc_comboBoxBaud.insets = new Insets(0, 0, 5, 5);
		gbc_comboBoxBaud.fill = GridBagConstraints.HORIZONTAL;
		gbc_comboBoxBaud.gridx = 2;
		gbc_comboBoxBaud.gridy = 1;
		frmOBDIIViewer.getContentPane().add(comboBoxBaud, gbc_comboBoxBaud);

		btnConnect = new JButton("Connect");
		btnConnect.setEnabled(false);
		btnConnect.addActionListener(new ActionListener()
		{
			public void actionPerformed(ActionEvent arg0)
			{
				// if the serial port is not connected, this is a connect action
				// if the serial port is connected, this is a disconnect action
				if (!serialConnected)
				{
					try
					{
						serialPort = new SerialPort((String) comboBoxPort.getSelectedItem());
						serialPort.openPort();
						serialPort.setParams(Integer.parseInt((String) comboBoxBaud.getSelectedItem()), SerialPort.DATABITS_8, SerialPort.STOPBITS_1, SerialPort.PARITY_NONE);
						serialPort.setEventsMask(SerialPort.MASK_RXCHAR);

						// add an event listener, which is the subclass at the
						// bottom of this file
						serialPort.addEventListener(new SerialReader());

						// change the button to be disconnect once we are
						// connected
						btnConnect.setText("Disconnect");
						serialConnected = true;
						comboBoxPort.setEnabled(false);
						comboBoxBaud.setEnabled(false);
					}
					catch (Exception exception)
					{
						System.err.println(exception.toString());
						serialPortClose();
					}
				}
				else
				{
					serialPortClose();
				}
			}
		});
		GridBagConstraints gbc_btnConnect = new GridBagConstraints();
		gbc_btnConnect.fill = GridBagConstraints.HORIZONTAL;
		gbc_btnConnect.insets = new Insets(0, 0, 5, 5);
		gbc_btnConnect.gridx = 3;
		gbc_btnConnect.gridy = 1;
		frmOBDIIViewer.getContentPane().add(btnConnect, gbc_btnConnect);

		panel = new JPanel();
		panel.setBorder(new TitledBorder(null, "Data", TitledBorder.LEADING, TitledBorder.TOP, null, null));
		GridBagConstraints gbc_panel = new GridBagConstraints();
		gbc_panel.gridwidth = 3;
		gbc_panel.insets = new Insets(0, 0, 5, 5);
		gbc_panel.fill = GridBagConstraints.BOTH;
		gbc_panel.gridx = 1;
		gbc_panel.gridy = 2;
		frmOBDIIViewer.getContentPane().add(panel, gbc_panel);
		GridBagLayout gbl_panel = new GridBagLayout();
		gbl_panel.columnWidths = new int[] { 10, 0, 0, 5, 0, 0 };
		gbl_panel.rowHeights = new int[] { 5, 0, 0, 0, 0, 0, 5, 0 };
		gbl_panel.columnWeights = new double[] { 1.0, 0.0, 0.0, 0.0, 1.0, Double.MIN_VALUE };
		gbl_panel.rowWeights = new double[] { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, Double.MIN_VALUE };
		panel.setLayout(gbl_panel);
		
		lblDelay = new JLabel("Delay:");
		GridBagConstraints gbc_lblDelay = new GridBagConstraints();
		gbc_lblDelay.anchor = GridBagConstraints.EAST;
		gbc_lblDelay.insets = new Insets(0, 0, 5, 5);
		gbc_lblDelay.gridx = 1;
		gbc_lblDelay.gridy = 1;
		panel.add(lblDelay, gbc_lblDelay);
		
		textFieldDelay = new JTextField();
		textFieldDelay.setHorizontalAlignment(SwingConstants.RIGHT);
		textFieldDelay.setEditable(false);
		GridBagConstraints gbc_textFieldDelay = new GridBagConstraints();
		gbc_textFieldDelay.insets = new Insets(0, 0, 5, 5);
		gbc_textFieldDelay.fill = GridBagConstraints.HORIZONTAL;
		gbc_textFieldDelay.gridx = 2;
		gbc_textFieldDelay.gridy = 1;
		panel.add(textFieldDelay, gbc_textFieldDelay);
		textFieldDelay.setColumns(10);
		
		lblSec = new JLabel("s");
		GridBagConstraints gbc_lblSec = new GridBagConstraints();
		gbc_lblSec.anchor = GridBagConstraints.WEST;
		gbc_lblSec.insets = new Insets(0, 0, 5, 5);
		gbc_lblSec.gridx = 3;
		gbc_lblSec.gridy = 1;
		panel.add(lblSec, gbc_lblSec);

		lblSpeed = new JLabel("Speed:");
		GridBagConstraints gbc_lblSpeed = new GridBagConstraints();
		gbc_lblSpeed.anchor = GridBagConstraints.EAST;
		gbc_lblSpeed.insets = new Insets(0, 0, 5, 5);
		gbc_lblSpeed.gridx = 1;
		gbc_lblSpeed.gridy = 2;
		panel.add(lblSpeed, gbc_lblSpeed);

		textFieldSpeed = new JTextField();
		textFieldSpeed.setEditable(false);
		textFieldSpeed.setHorizontalAlignment(SwingConstants.RIGHT);
		GridBagConstraints gbc_textFieldSpeed = new GridBagConstraints();
		gbc_textFieldSpeed.insets = new Insets(0, 0, 5, 5);
		gbc_textFieldSpeed.fill = GridBagConstraints.HORIZONTAL;
		gbc_textFieldSpeed.gridx = 2;
		gbc_textFieldSpeed.gridy = 2;
		panel.add(textFieldSpeed, gbc_textFieldSpeed);
		textFieldSpeed.setColumns(10);
		
		lblKmh = new JLabel("km/h");
		GridBagConstraints gbc_lblKmh = new GridBagConstraints();
		gbc_lblKmh.anchor = GridBagConstraints.WEST;
		gbc_lblKmh.insets = new Insets(0, 0, 5, 5);
		gbc_lblKmh.gridx = 3;
		gbc_lblKmh.gridy = 2;
		panel.add(lblKmh, gbc_lblKmh);
		
		lblDistance = new JLabel("Distance:");
		GridBagConstraints gbc_lblDistance = new GridBagConstraints();
		gbc_lblDistance.anchor = GridBagConstraints.EAST;
		gbc_lblDistance.insets = new Insets(0, 0, 5, 5);
		gbc_lblDistance.gridx = 1;
		gbc_lblDistance.gridy = 3;
		panel.add(lblDistance, gbc_lblDistance);
		
		textFieldDistance = new JTextField();
		textFieldDistance.setHorizontalAlignment(SwingConstants.RIGHT);
		textFieldDistance.setEditable(false);
		GridBagConstraints gbc_textFieldDistance = new GridBagConstraints();
		gbc_textFieldDistance.insets = new Insets(0, 0, 5, 5);
		gbc_textFieldDistance.fill = GridBagConstraints.HORIZONTAL;
		gbc_textFieldDistance.gridx = 2;
		gbc_textFieldDistance.gridy = 3;
		panel.add(textFieldDistance, gbc_textFieldDistance);
		textFieldDistance.setColumns(10);
		
		lblMetres = new JLabel("m");
		GridBagConstraints gbc_lblMetres = new GridBagConstraints();
		gbc_lblMetres.anchor = GridBagConstraints.WEST;
		gbc_lblMetres.insets = new Insets(0, 0, 5, 5);
		gbc_lblMetres.gridx = 3;
		gbc_lblMetres.gridy = 3;
		panel.add(lblMetres, gbc_lblMetres);

		lblRate = new JLabel("Rate:");
		GridBagConstraints gbc_lblRate = new GridBagConstraints();
		gbc_lblRate.anchor = GridBagConstraints.EAST;
		gbc_lblRate.insets = new Insets(0, 0, 5, 5);
		gbc_lblRate.gridx = 1;
		gbc_lblRate.gridy = 4;
		panel.add(lblRate, gbc_lblRate);

		textFieldRate = new JTextField();
		textFieldRate.setEditable(false);
		textFieldRate.setHorizontalAlignment(SwingConstants.RIGHT);
		GridBagConstraints gbc_textFieldRate = new GridBagConstraints();
		gbc_textFieldRate.insets = new Insets(0, 0, 5, 5);
		gbc_textFieldRate.fill = GridBagConstraints.HORIZONTAL;
		gbc_textFieldRate.gridx = 2;
		gbc_textFieldRate.gridy = 4;
		panel.add(textFieldRate, gbc_textFieldRate);
		textFieldRate.setColumns(10);
		
		lblHz = new JLabel("Hz");
		GridBagConstraints gbc_lblHz = new GridBagConstraints();
		gbc_lblHz.anchor = GridBagConstraints.WEST;
		gbc_lblHz.insets = new Insets(0, 0, 5, 5);
		gbc_lblHz.gridx = 3;
		gbc_lblHz.gridy = 4;
		panel.add(lblHz, gbc_lblHz);
		
		chckbxReversingDetection = new JCheckBox("Reversing Detection");
		GridBagConstraints gbc_chckbxReversingDetection = new GridBagConstraints();
		gbc_chckbxReversingDetection.gridwidth = 3;
		gbc_chckbxReversingDetection.insets = new Insets(0, 0, 5, 5);
		gbc_chckbxReversingDetection.gridx = 1;
		gbc_chckbxReversingDetection.gridy = 5;
		panel.add(chckbxReversingDetection, gbc_chckbxReversingDetection);

		textArea = new JTextArea();
		textArea.setEditable(false);
		JScrollPane scrollPane = new JScrollPane(textArea);
		GridBagConstraints gbc_textArea = new GridBagConstraints();
		gbc_textArea.gridwidth = 3;
		gbc_textArea.insets = new Insets(0, 0, 5, 5);
		gbc_textArea.fill = GridBagConstraints.BOTH;
		gbc_textArea.gridx = 1;
		gbc_textArea.gridy = 3;
		frmOBDIIViewer.getContentPane().add(scrollPane, gbc_textArea);
		
		menuBar = new JMenuBar();
		frmOBDIIViewer.setJMenuBar(menuBar);
		
		mnFile = new JMenu("File");
		menuBar.add(mnFile);
		
		mntmExit = new JMenuItem("Exit");
		mntmExit.addActionListener(new ActionListener() 
		{
			public void actionPerformed(ActionEvent arg0) 
			{
				frmOBDIIViewer.dispose();
				System.exit(0);
			}
		});
		mnFile.add(mntmExit);
		
		mnHelp = new JMenu("Help");
		menuBar.add(mnHelp);
		
		mntmAbout = new JMenuItem("About");
		mntmAbout.addActionListener(new ActionListener() 
		{
			public void actionPerformed(ActionEvent arg0) 
			{
				if(frmAbout == null)
				{
					frmAbout = new JFrame();
					frmAbout.setTitle("About");
					frmAbout.setBounds(100, 100, 350, 250);
					frmAbout.setDefaultCloseOperation(JFrame.DISPOSE_ON_CLOSE);
					GridBagLayout gridBagLayout = new GridBagLayout();
					gridBagLayout.columnWidths = new int[]{0, 0, 0, 0};
					gridBagLayout.rowHeights = new int[]{0, 0, 0, 0, 0, 0, 0, 0, 0};
					gridBagLayout.columnWeights = new double[]{1.0, 0.0, 1.0, Double.MIN_VALUE};
					gridBagLayout.rowWeights = new double[]{1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, Double.MIN_VALUE};
					frmAbout.getContentPane().setLayout(gridBagLayout);
					
					JLabel lblNewLabel = new JLabel("");
					lblNewLabel.setIcon(new ImageIcon(Main.class.getResource("/com/advancednavigation/images/an_logo.png")));
					GridBagConstraints gbc_lblNewLabel = new GridBagConstraints();
					gbc_lblNewLabel.insets = new Insets(0, 0, 10, 5);
					gbc_lblNewLabel.gridx = 1;
					gbc_lblNewLabel.gridy = 1;
					frmAbout.getContentPane().add(lblNewLabel, gbc_lblNewLabel);
					
					JLabel lblObdii = new JLabel("OBDII Odometer Viewer Version 1.0");
					GridBagConstraints gbc_lblObdii = new GridBagConstraints();
					gbc_lblObdii.insets = new Insets(0, 0, 5, 5);
					gbc_lblObdii.gridx = 1;
					gbc_lblObdii.gridy = 2;
					frmAbout.getContentPane().add(lblObdii, gbc_lblObdii);
					
					Date date;
					try
					{
						date = new Date(Main.class.getResource("Main.class").openConnection().getLastModified());
					}
					catch (Exception exception)
					{
						date = new GregorianCalendar(2014, 0, 0).getTime();
					}
					JLabel lblBuildDate = new JLabel("Build Date: " + date);
					GridBagConstraints gbc_lblBuildDate = new GridBagConstraints();
					gbc_lblBuildDate.insets = new Insets(0, 0, 5, 5);
					gbc_lblBuildDate.gridx = 1;
					gbc_lblBuildDate.gridy = 3;
					frmAbout.getContentPane().add(lblBuildDate, gbc_lblBuildDate);
					
					JLabel lblCopyrightc = new JLabel("Copyright (c) 2014");
					GridBagConstraints gbc_lblCopyrightc = new GridBagConstraints();
					gbc_lblCopyrightc.insets = new Insets(0, 0, 5, 5);
					gbc_lblCopyrightc.gridx = 1;
					gbc_lblCopyrightc.gridy = 4;
					frmAbout.getContentPane().add(lblCopyrightc, gbc_lblCopyrightc);
					
					JLabel lblAdvancedNavigationPty = new JLabel("Advanced Navigation Pty Ltd");
					GridBagConstraints gbc_lblAdvancedNavigationPty = new GridBagConstraints();
					gbc_lblAdvancedNavigationPty.insets = new Insets(0, 0, 5, 5);
					gbc_lblAdvancedNavigationPty.gridx = 1;
					gbc_lblAdvancedNavigationPty.gridy = 5;
					frmAbout.getContentPane().add(lblAdvancedNavigationPty, gbc_lblAdvancedNavigationPty);
					
					JLabel lblHttpwwwadvancednavigationcomau = new JLabel("http://www.advancednavigation.com.au");
					GridBagConstraints gbc_lblHttpwwwadvancednavigationcomau = new GridBagConstraints();
					gbc_lblHttpwwwadvancednavigationcomau.insets = new Insets(0, 0, 5, 5);
					gbc_lblHttpwwwadvancednavigationcomau.gridx = 1;
					gbc_lblHttpwwwadvancednavigationcomau.gridy = 6;
					frmAbout.getContentPane().add(lblHttpwwwadvancednavigationcomau, gbc_lblHttpwwwadvancednavigationcomau);
					frmAbout.setVisible(true);
				}
				else
				{
					frmAbout.setVisible(true);
					frmAbout.toFront();
				}
			}
		});
		mnHelp.add(mntmAbout);

		// create a thread to update the available serials ports once a second
		// it is done in a new thread because under windows it can block for up
		// to 10 seconds
		Thread portScannerThread = new Thread(new PortScanner());
		portScannerThread.start();
	}

	public class PortScanner implements Runnable
	{
		public void run()
		{
			while (true)
			{
				if (!serialConnected)
				{
					SwingUtilities.invokeLater(new Runnable()
					{
						public void run()
						{
							if (!serialConnected)
							{
								String[] portNames = SerialPortList.getPortNames();
								int comboLength = comboBoxPort.getItemCount();
								if (!serialPortsAvailable) comboLength = 0;
								Boolean portsChanged = portNames.length != comboLength;
								if(!portsChanged)
								{
									for(int i = 0; i < portNames.length; i++)
									{
										if(portNames[i].compareTo((String)comboBoxPort.getItemAt(i)) != 0)
										{
											portsChanged = true;
											break;
										}
									}
								}
								if (portsChanged)
								{
									comboBoxPort.removeAllItems();
									for (int i = 0; i < portNames.length; i++)
									{
										comboBoxPort.addItem(portNames[i]);
									}
									if (portNames.length == 0)
									{
										comboBoxPort.addItem("No Serial Ports");
										comboBoxPort.setSelectedIndex(0);
										comboBoxPort.setEnabled(false);
										btnConnect.setEnabled(false);
										serialPortsAvailable = false;
									}
									else
									{
										comboBoxPort.setSelectedIndex(0);
										comboBoxPort.setEnabled(true);
										btnConnect.setEnabled(true);
										serialPortsAvailable = true;
									}
								}
							}
						}
					});
				}
				else
				{
					textFieldRate.setText(Integer.toString(packetCount));
					packetCount = 0;
				}
				try
				{
					Thread.sleep(1000);
				}
				catch (InterruptedException e)
				{
					// TODO Auto-generated catch block
					e.printStackTrace();
				}
			}
		}
	}

	private void serialPortClose()
	{
		try
		{
			serialPort.removeEventListener();
			serialPort.closePort();
		}
		catch (SerialPortException e)
		{
			// TODO Auto-generated catch block
			e.printStackTrace();
		}
		btnConnect.setText("Connect");
		serialConnected = false;
		comboBoxPort.setEnabled(true);
		comboBoxBaud.setEnabled(true);
	}

	/*
	 * This class is the event listener for received serial data
	 * Whenever serial data is received the serialEvent is called
	 * which adds the received data to the decoders buffer and then
	 * checks whether any packets can be decoded.
	 */
	public class SerialReader implements SerialPortEventListener
	{
		ANPacketDecoder packetDecoder;

		public SerialReader()
		{
			packetDecoder = new ANPacketDecoder();
		}

		public void serialEvent(SerialPortEvent event)
		{
			if (event.isRXCHAR())
			{
				try
				{
					byte[] buffer = serialPort.readBytes();
					if (buffer != null)
					{
						for (int i = 0; i < buffer.length; i++)
						{
							if (packetDecoder.bufferLength < packetDecoder.buffer.length)
							{
								packetDecoder.buffer[packetDecoder.bufferLength++] = buffer[i];
							}
						}
						
						ANPacket packet = null;
						while ((packet = packetDecoder.packetDecode()) != null)
						{
							switch (packet.id)
							{
								case ANPacket.PACKET_ID_EXTERNAL_ODOMETER:
									if (packet.length == 13)
									{
										final ANPacket67 anPacket67 = new ANPacket67(packet);
										packetCount++;

										SwingUtilities.invokeLater(new Runnable()
										{
											public void run()
											{
												textFieldDelay.setText(String.format("%.3f", anPacket67.delay));
												textFieldSpeed.setText(String.format("%.3f", anPacket67.speed*3.6f));
												textFieldDistance.setText(String.format("%.3f", anPacket67.distanceTravelled));
												chckbxReversingDetection.setSelected(anPacket67.reverseDetectionSupported);
											}
										});
									}
									break;
								default:
									break;
							}
						}
					}
				}
				catch (SerialPortException exception)
				{
					System.err.println(exception.toString());
				}
			}
		}
	}
}
