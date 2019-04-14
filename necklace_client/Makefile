all: client

client: client.c  Makefile
	gcc -std=c99  -Wall -pedantic -o client client.c -g -lm -lrt  -Wno-unused-but-set-variable -Wno-unused-variable
	#gcc -o client client.c -g -lm -lrt -Wall -Wno-unused-but-set-variable -Wno-unused-variable

record: client.c
	./client -record out.dat
	chmod u+r out.dat

play: client.c
	./client -play out.dat


go: client
	./client


clean:
	rm -rf *~
	rm -rf client
