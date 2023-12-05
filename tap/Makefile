tap: tap.cpp
	g++ tap.cpp -std=c++23 -o tap

test: tap
	PATH="${PATH}:$(pwd)" ip netns exec test tap $()
