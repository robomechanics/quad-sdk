TARGET   := examples_2D 


ARCHITECTURE=`/bin/uname -m`

# Choose a matching distribution below:
#DISTRO :=../lib_centos7.8.2003_${ARCHITECTURE}
#DISTRO :=../lib_centos6.4_${ARCHITECTURE}
#DISTRO :=../lib_ubuntu14.04_${ARCHITECTURE}
#DISTRO :=../lib_ubuntu18.04_${ARCHITECTURE}
DISTRO :=../lib_ubuntu21.04_${ARCHITECTURE}
#DISTRO :=../lib_APPLE
#DISTRO :=../lib_raspberry_armv6l
#DISTRO :=../lib_raspberry_armv7l


# Test with -std=c++98 to ensure backwards compatiblity

CXX      := g++
ifeq ($(DISTRO),../lib_APPLE)
	CXXFLAGS := -g -Wextra -Wall -Wno-long-long -pedantic-errors  -I../include_fade2d 
	LIBS     := -lfade2d -rpath @executable_path/${DISTRO} -Wl,
	LNKFLAGS := -L${DISTRO}
else

	CXXFLAGS := -g -Wextra -Wall -Wno-long-long -pedantic-errors -I../include_fade2d -L${DISTRO}
	LIBS     := -lfade2d -Wl,-rpath=${DISTRO}
endif


EXT      := cpp
BUILDDIR := build

override BUILDDIR := $(strip $(BUILDDIR))
SOURCES  := $(wildcard *.$(EXT))
OBJECTS  := $(patsubst %.$(EXT), $(BUILDDIR)/%.o, $(SOURCES))
DEPS     := $(patsubst %.$(EXT), $(BUILDDIR)/%.dep, $(SOURCES))

.PHONY: all
all: $(TARGET)

$(TARGET): $(OBJECTS) $(DEPS)
	$(CXX) $(CXXFLAGS) $(LNKFLAGS) -o $(TARGET) $(OBJECTS) $(LIBS)

ifneq ($(MAKECMDGOALS), clean)
-include $(DEPS)
endif

$(OBJECTS): $(BUILDDIR)/%.o: %.$(EXT) $(BUILDDIR)/%.dep $(BUILDDIR)/.tag
	$(CXX) $(CXXFLAGS) -c $< -o $@

$(DEPS): $(BUILDDIR)/%.dep: %.$(EXT) $(BUILDDIR)/.tag
	mkdir -p $(dir $(@))
	$(CXX) $(CXXFLAGS) -MM $< -MT $@ -MT $(<:.$(EXT)=.o) -o $@

%.tag:
	mkdir -p $(dir $(@))
	touch $@

.PHONY: clean
clean:
	pwd
	echo $(LIBS)
	rm -rf $(BUILDDIR) $(TARGET) *.ps *.list out0.txt out1.txt out2.txt 


