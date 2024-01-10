# cpp-data-channel-client-reliability-tests

This example tests the reliability of the data channel.

## How to use

1. Build the example.
```bash
cd ../..
mkdir build
cd build
cmake ..
cmake --build . --config Release|Debug
```

2. Start the signaling server.
```bash
./start_server.bash
```

3. Start a master client.
```bash
cd ../../build/bin/Release
./CppDataChannelReliabilityTests http://localhost:8080 master abc true
```

4. Start a slave client.
```bash
cd ../../build/bin/Release
./CppDataChannelReliabilityTests http://localhost:8080 slave abc false
```
