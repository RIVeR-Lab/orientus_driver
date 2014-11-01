all: packet_example

packet_example: rs232/rs232.c rs232/rs232.h an_packet_protocol.c an_packet_protocol.h orientus_packets.c orientus_packets.h packet_example.c
	gcc -pedantic -Wall -o packet_example rs232/rs232.c an_packet_protocol.c orientus_packets.c packet_example.c
	
clean:
	rm packet_example
