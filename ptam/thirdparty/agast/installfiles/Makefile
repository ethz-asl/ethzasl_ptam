# Minimalistic Makefile for the AGAST demo package
# author: Elmar Mair
# date: 06/2010 
# license: GPL v3

####################################################
# please adapt following lines to your environment # 
####################################################
#OPENCV_INCLUDE_PATH = /home/slynen/ros_cturtle/stacks/vision_opencv/opencv2/opencv/include
#OPENCV_LIBRARY_PATH = /usr/lib

########################################################
# the following lines should not be edited by the user #
########################################################
CXX = g++
PROG = demo
OBJ = agast5_8.o agast5_8_nms.o agast7_12s.o agast7_12s_nms.o	agast7_12d.o agast7_12d_nms.o oast9_16.o oast9_16_nms.o nonMaximumSuppression.o 
#-I$(OPENCV_INCLUDE_PATH)
CXXFLAGS += -DHAVE_OPENCV -O3 -Wall -Wextra $(AGAST_INCLUDE) $(OPENCV_INCLUDE)
#-L$(OPENCV_LIBRARY_PATH)
LDFLAGS += -lcv -lhighgui -lcxcore

all: $(PROG)

$(PROG) : ../src/$(PROG).cc $(OBJ)
	$(CXX) $(CXXFLAGS) ../src/$(PROG).cc $(LDFLAGS) $(OBJ) -o $@

%.o : ../src/%.c
	$(CXX) $(CXXFLAGS) -c $<
        
clean:
	-@rm *.o $(PROG)

