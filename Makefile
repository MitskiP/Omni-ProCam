CC = g++
LD = g++

SRCDIR = src
BINDIR = bin
OBJDIR = obj

LIBS   = `pkg-config opencv --cflags --libs` -lpthread
CFLAGS  += -Wall -Wextra -pedantic -Wno-unused-parameter -std=c++11 -c $(LIBS)
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

EXECUTABLE = aruco
TARGET = $(BINDIR)/$(EXECUTABLE)

all: $(SOURCES) $(TARGET) $(TARGET2)

$(TARGET): $(OBJECTS) | $(BINDIR)
	$(CC) $(LDFLAGS) $^ -o $@

$(OBJDIR)/%.o: $(SRCDIR)/%.cpp | $(OBJDIR)
	$(CC) $(CFLAGS) -o $@ $<

$(BINDIR) $(OBJDIR):
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

