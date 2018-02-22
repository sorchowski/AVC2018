
CC=g++
LIBS=-lnmea -lroscpp_serialization -lroscpp -lrosconsole -lrostime
LFLAGS=-L/opt/ros/kinetic/lib
#SHARED==Wl,-rpath,/opt/ros/kinetic/lib
INCLUDES=-I../../../lib/common -I/opt/ros/kinetic/include
SOURCES=node_read_gps.cpp
OBJECTS=$(SOURCES:.cpp=.o)
EXECUTABLE=node_read_gps

# $@ is the left-hand side of the :
# $< is the right-hand side of the :

%.o: %.cpp
	$(CC) $(INCLUDES) -c -o $@ $<

$(EXECUTABLE): $(OBJECTS)
	$(CC) $(OBJECTS) -o $@ -v $(LFLAGS) $(LIBS)

clean:
	rm -f $(OBJECTS) $(EXECUTABLE)

