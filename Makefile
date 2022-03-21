PLATFORM = x86_64
#PLATFORM = x86_64 or arm64

INCLUDES += -I./

C_FLAGS += -Wall

LIBS = -L ./lib/$(PLATFORM) -lsagittarius_sdk -lpthread -lboost_system -lboost_thread

LD = $(MVTOOL_PREFIX)g++
CC = $(MVTOOL_PREFIX)g++ $(INCLUDES) $(C_FLAGS) 


sagittarius_example :

	$(CC) -lpthread -o sagittarius_example sagittarius_example.cpp  $(LIBS)

clean:
	-$(RM) -f sagittarius_example
	-$(RM) -f *.o

install:

	sudo cp ./lib/$(PLATFORM)/libsagittarius_sdk.so /usr/lib/

