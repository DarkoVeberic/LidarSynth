ifdef ROOTSYS
  ROOTBIN := $(ROOTSYS)/bin
else
  ROOTBIN := /usr/bin
endif

CPPFLAGS := -I. $(shell $(ROOTBIN)/root-config --cflags)
#CXXFLAGS := -Wall -Wextra -ggdb3 -O9 -march=native -mfpmath=sse -msse3 -ftree-vectorize -pipe -DNDEBUG -fPIC
CXXFLAGS := -Wall -Wextra -ggdb3 -O2 -pipe -fPIC
#CXXFLAGS := -Wall -Wextra -ggdb3 -O0 -fno-inline -pipe
LDFLAGS := -pipe $(shell $(ROOTBIN)/root-config --ldflags --libs) -lMinuit -lMinuit2 -lMathMore -lboost_iostreams

LIBFLAGS := -fPIC -shared

LIDARSRCS := $(wildcard lidar/*.cc) utl/Math.cc
LIDAROBJS := $(LIDARSRCS:.cc=.o)

MAINSRCS := $(wildcard *.cc)

OBJS := $(LIDARSRCS:.cc=.o) $(MAINSRCS:.cc=.o)
DEPS := $(OBJS:.o=.P)

EXES := dead_time_simulation libLidar.so MaxLikeSynthesisMinuit2

define cpp_compile_with_dependency_creation
  $(COMPILE.cc) -MD -o $@ $<
  @sed -e 's|.*:|$*.o:|' <$*.d >$*.P
  @sed -e 's/.*://' -e 's/\\$$//' <$*.d | fmt -1 | \
    sed -e 's/^ *//' -e 's/$$/:/' >>$*.P
  @rm -f $*.d
endef

define link_rule
  $(CXX) $^ $(LDFLAGS) -o $@
endef

define lidar_link_rule
  $(CXX) $^ $(LDFLAGS) -L. -lLidar -o $@
endef

define ld_shared_library
  $(CXX) -shared $(CXXFLAGS) $^ -o $@
endef

%.o: %.cc
	$(call cpp_compile_with_dependency_creation)
	
%.o: %.cxx
	$(call cpp_compile_with_dependency_creation)

%: %.o
	$(call link_rule)

.PHONY: all clean depend

all: $(EXES)

dead_time_simulation: dead_time_simulation.o

libLidar.so: $(LIDAROBJS)
	$(call ld_shared_library)

MaxLikeSynthesis: MaxLikeSynthesis.o libLidar.so
	$(call lidar_link_rule)

MaxLikeSynthesisMinuit2: MaxLikeSynthesisMinuit2.o libLidar.so
	$(call lidar_link_rule)

clean:
	- rm $(EXES) $(OBJS) $(DEPS)

-include $(DEPS)
