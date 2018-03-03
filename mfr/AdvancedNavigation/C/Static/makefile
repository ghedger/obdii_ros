all: packet_example

packet_example: an_packet_protocol.c an_packet_protocol.h obdii_odometer_data_dump.h obdii_odometer_packets.c obdii_odometer_packets.h packet_example.c
	gcc -pedantic -Wall -o packet_example an_packet_protocol.c obdii_odometer_packets.c packet_example.c
	
clean:
	rm packet_example
