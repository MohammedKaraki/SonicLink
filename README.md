# SonicLink

### Prerequisites:
- The Boost library (circular_buffer.hpp, lockfree/spsc_queue.hpp)
- The PortAudio library (portaudio.h)
- The fmtlib library (fmt/format.h, fmt/color.h)
- Version >=10 of g++

On Ubuntu, you can obtain these requirements with:
```
sudo apt install libboost1.71-dev libasound2-dev libportaudio2 portaudio19-dev libfmt-dev g++-10
```

### Compilation
Compile using the following:
```
g++-10 -O3 -std=c++20 -o send.x send.cpp -lpthread -lasound -lportaudio
g++-10 -O3 -std=c++20 -o recv.x recv.cpp -lpthread -lasound -lportaudio
```
