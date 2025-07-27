CXX := g++
CXXFLAGS := -std=c++17 -isystem/opt/homebrew/opt/googletest/include -Isrc -Itests/gtest/mocks -pthread
LDFLAGS := -L/opt/homebrew/opt/googletest/lib -lgtest -lgtest_main

test: tests/gtest/turret_test.o src/turret.o tests/gtest/mocks/Arduino.o tests/gtest/mocks/IRremote.o tests/gtest/mocks/Servo.o
	$(CXX) $(CXXFLAGS) -o $@ $^ $(LDFLAGS)

%.o: %.cpp
	$(CXX) $(CXXFLAGS) -c $< -o $@

clean:
	rm -f tests/gtest/*.o src/*.o tests/gtest/mocks/*.o test
