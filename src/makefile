sources := send.cpp recv.cpp main.cpp packet.cpp binary.cpp
objects := $(sources:.cpp=.o)

link_flags := 
compile_flags := -g -O0 -std=c++20 -Wall -Wpedantic -Wextra

CC := clang++

x: $(objects)
	$(CC) $(link_flags) $^ -o $@  -lpthread -lportaudio -lasound -lfmt

$(objects): %.o:%.cpp
	$(CC) $(compile_flags) -c $< -o $@

depend: .depend

.depend: $(sources)
	rm -f ./.depend
	$(CC) $(compile_flags) -MM $^ -MF  ./.depend;

include .depend

.PHONY: clean
clean:
	rm -f *.o
