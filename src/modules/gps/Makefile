
CC=g++
LIBS=-lnmea
INCLUDES=-I../../../lib/common
SOURCES=node_read_gps.cpp
OBJECTS=$(SOURCES:.cpp=.o)
EXECUTABLE=node_read_gps

# $@ is the left-hand side of the :
# $< is the right-hand side of the :

%.o: %.cpp
	$(CC) $(INCLUDES) -c -o $@ $<

$(EXECUTABLE): $(OBJECTS)
	$(CC) $(OBJECTS) -o $@ -v $(LIBS)
