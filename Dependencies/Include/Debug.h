#ifndef TRIANGLE_DEBUG_H
#define TRIANGLE_DEBUG_H

#include "Common.h"

class Camera;
class Mesh;

void EditTransform(Camera* cam, glm::mat4* cameraView, glm::mat4* cameraProjection, Mesh* mesh);

std::string getTabs(int n);


class Timer {
public:
	bool printElapsedTimeInDestructor;
	std::chrono::steady_clock::time_point then;
	std::string label;

	bool paused{};
	std::chrono::steady_clock::time_point pausedAt{};
	std::chrono::nanoseconds pausedDuration{};

	Timer(bool printElapsedTimeInDestructor, std::string label);
	void pause();
	void unpause();
	void printTime(std::string timestampLabel);
	~Timer();
};

#endif //TRIANGLE_DEBUG_H
