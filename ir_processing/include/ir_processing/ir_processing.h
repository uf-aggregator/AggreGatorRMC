#ifndef IR_PROCESSING_H
#define IR_PROCESSING_H


class IrProcessing {
	private:
		static const char* serviceNm;
		static const float i2c_max;
		static const float ir_max_out;
		static const float ir_min_dist;
		static const float ir_max_dist;
		static const int i2c_addr;
		static const int i2c_size;
		IrProcessing();
		~IrProcessing(){
		}

	public:
		static uint getValue();
		static int getCentimeterI2CMap(int value);
		static int getCentimeters();
		static float getMeters();
		static float getFeet();
};


#endif
