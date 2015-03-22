#ifndef SEEK_TARGET_BEHAVIOR_H 
#define SEEK_TARGET_BEHAVIOR_H

class SeekTargetBehavior {
	private:
		int lastSeen; //-1 null, 0 left, 1 right

	public:
		SeekTargetBehavior();
		~SeekTargetBehavior(){}

		public void turnLeft();
		public void turnRight();
		public void find();
		public void test();
};

#endif