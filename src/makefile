CC = g++
BINDIR = ../bin
CFLAGS =  -I. -I/usr/local/include/player-2.1 -Wall
CPPFLAGS = $(CFLAGS)
LFLAGS = -lLaser -lMage -lm -lpthread -lplayerc++ 
DEPS = motion_utilities.h Particle.h p_filter.h ppm.h #ArrayList.cpp

./%o : %.c $(DEPS)
	$(CC) -c -o $@ $< $(CFLAGS)

./%o : %.cpp $(DEPS)
	$(CC) -c -o $@ $< $(CFLAGS)

all: fastSLAM
#Landscape.o
fastSLAM : state.o motion_utilities.o ArrayList.o Particle.o p_filter.o ppm.o display.o
	$(CC)  -o $(BINDIR)/fastSLAM $^ $(LFLAGS) -L/usr/X11R6/lib -lX11 -lipc -lgcm

clean:
	rm -f *.o *~
