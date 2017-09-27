
EXE=exercise1_mac
SOURCES = $(wildcard *.cpp)
OBJECTS = $(patsubst %.cpp,%.o,$(SOURCES))

# -L... needed for OpenGL with OSX 10.5
LDFLAGS    = -framework CoreServices -framework glut -framework OpenGL -L/System/Library/Frameworks/OpenGL.framework/Versions/A/Libraries
CFLAGS     = -c -O2 -DMACVERSION=1


default: $(EXE)

$(EXE): $(OBJECTS)
	$(CXX) -o $(EXE) $(LDFLAGS) $(OBJECTS)

%.o : %.cpp
	$(CXX) $(CFLAGS) -o $@ $<

clean:
	 rm -f *.o $(EXE)

