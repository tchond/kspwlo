CC      = /usr/local/bin/g++-10
LDFLAGS = -I/usr/local/Cellar/boost/1.73.0/include -I/opt/local/include
LLIBFLAGS = -L/usr/local/Cellar/boost/1.73.0/
LINKFLAGS = -l boost_thread-mt -lboost_system
CFLAGS  = -g -fmessage-length=0 -c -Wall -Wextra -pedantic -Wredundant-decls -Wdisabled-optimization -Wctor-dtor-privacy -Wnon-virtual-dtor -Woverloaded-virtual -Wsign-promo -Wold-style-cast -Werror=return-type -DLINUX -std=c++11 -Ofast  $(LLIBFLAGS) $(LDFLAGS) $(LINKFLAGS) 
MODEL = model/graph.cpp
TOOLS = tools/dijkstra.cpp tools/astar.cpp algorithms/completeness.cpp 
ALGORITHMS = algorithms/skyline.cpp algorithms/onepass.cpp algorithms/multipass.cpp algorithms/onepass_plus.cpp algorithms/svp_plus.cpp algorithms/esx.cpp
SOURCES = $(MODEL) $(TOOLS) $(ALGORITHMS)  main.cpp
#
OBJECTS = $(SOURCES:.cpp=.o)
#

all: $(OBJECTS)
	$(CC) $(OBJECTS) -o ./run.exec

.cpp.o:
	$(CC) $(CFLAGS) $< -o $@ 

.cc.o:
	$(CC) $(CFLAGS) $< -o $@

clean:
	rm -rf *.o
	rm -rf model/*.o
	rm -rf tools/*.o
	rm -rf algorithms/*.o
	rm *.exec
