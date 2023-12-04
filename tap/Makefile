tap: tap.cpp
	g++ tap.cpp -std=c++23 -o tap

test: tap
	PATH=$(pwd) tap && ip netns exec test tap
