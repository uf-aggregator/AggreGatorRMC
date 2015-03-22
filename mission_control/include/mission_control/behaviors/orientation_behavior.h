#ifndef ORIENTATION_BEHAVIOR_H 
#define ORIENTATION_BEHAVIOR_H

class OrientationBehavior {
	private:
		int lastSeen; //-1 null, 0 left, 1 right

	public:
		OrientationBehavior();
		~OrientationBehavior(){}

		void turnLeft();
		void turnRight();
		float* find();
		void test();
};

#endif