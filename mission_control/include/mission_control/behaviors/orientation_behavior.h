#ifndef ORIENTATION_BEHAVIOR_H 
#define ORIENTATION_BEHAVIOR_H

enum Direction {
	NONE = -1,
	LEFT = 0,
	RIGHT
};

class OrientationBehavior {
	private:
		Direction lastSeen; //-1 null, 0 left, 1 right
		float x1, y1; //first pair of coords
		float x2, y2; //second pair of coords
		float width, height; //resolution of the camera

	public:
		OrientationBehavior();
		~OrientationBehavior(){}

		void turn();
		void find();
		void orient();
};

#endif