CC = g++
LD = g++

SRCDIR = src
BINDIR = bin
OBJDIR = obj
SPHEREOBJ = $(OBJDIR)/Sphere

LIBS   = `pkg-config opencv4 --cflags --libs` -lpthread
CFLAGS  += -Wall -Wextra -pedantic -Wno-unused-parameter -Wno-unused-command-line-argument -std=c++11 -c $(LIBS)
LDFLAGS += $(LIBS)

ifeq ($(DEBUG),1)
CFLAGS += -O0 -no-pie -g -pg -DDEBUG -D_DEBUG
LDFLAGS += -O0 -no-pie -g -pg -DDEBUG -D_DEBUG
else
CFLAGS += -O3 -flto -DRELEASE
LDFLAGS += -O3 -flto -DRELEASE
endif

SOURCES = $(wildcard $(SRCDIR)/*.cpp) $(wildcard $(SRCDIR)/*/*.cpp)
HEADERS = $(wildcard $(SRCDIR)/*.h) $(wildcard $(SRCDIR)/*/*.h)
OBJECTS = $(patsubst $(SRCDIR)/%.cpp,$(OBJDIR)/%.o,$(SOURCES))

EXECUTABLE = RectFollower
TARGET = $(BINDIR)/$(EXECUTABLE)

EXECUTABLE2 = BoardTracker
TARGET2 = $(BINDIR)/$(EXECUTABLE2)

all: $(SOURCES) $(TARGET) $(TARGET2)

$(TARGET): $(filter-out obj/$(EXECUTABLE2).o,$(OBJECTS)) | $(BINDIR)
	$(CC) $(LDFLAGS) $^ -o $@

$(TARGET2): $(filter-out obj/$(EXECUTABLE).o,$(OBJECTS)) | $(BINDIR)
	$(CC) $(LDFLAGS) $^ -o $@

$(OBJDIR)/%.o: $(SRCDIR)/%.cpp | $(OBJDIR) $(SPHEREOBJ)
	$(CC) $(CFLAGS) -o $@ $<

$(BINDIR) $(OBJDIR) $(SPHEREOBJ):
	mkdir -p $@

module = uvcvideo
module:
	lsmod | grep $(module) &>/dev/null || sudo modprobe $(module)

run: module $(TARGET)
	$(TARGET) ${ARGS}

clean:
	rm -rf $(BINDIR) $(OBJDIR)

.PHONY:
	clean run all

