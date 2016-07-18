.SUFFIXES : .cpp .o
CC= g++
#INC = 

LIBS = -L/usr/include/ -lglut -lGLU -lGL -lm
CFLAGS = -g -c -Wall -Wextra $(INC) 

OBJS = PTPopenGL.o KalmanFilter.o PTPSystem.o PTPCore.o PTPUtil.o 
#OBJS = ptp_main_test.o PTPSystem.o PTPCore.o PTPUtil.o 


SRCS = $(OBJS:.o=.cpp)
TARGET = terra
all : $(TARGET)
$(TARGET) : $(OBJS)
	$(CC) -o $@ $(OBJS) $(LIBS) 
	#$(CC) -o $(CFLAGS) $@ $(OBJS) $(LIBS) 
#dep :
#	gccmakedep $(INC) $(SRCS)
clean :
	rm -rf $(OBJS) $(TARGET) core
new : 
	$(MAKE) clean
	$(MAKE) 
