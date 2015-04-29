#ifndef IR_READER_H
#define IR_READER_H

class IrReader {
	private:
		/* Static data members */
		static const char* serviceNm;
		static const float i2c_max;
		static const float ir_max_out;
		static const float ir_min_dist;
		static const float ir_max_dist;
		
		/* Instance data members */
		int i2c_addr;
		int i2c_size;

	public:
		/* Constructor */
		IrReader(int addr, int size);
		~IrReader(){}
		
		/* Instance methods */
		uint getValue();
		int getCentimeters();
		float getMeters();
		float getFeet();

		/* Static utility methods */
		static uint getValueOf(int addr, int size);
		static int getCentimetersOf(int addr, int size);
		static float getMetersOf(int addr, int size);
		static float getFeetOf(int addr, int size);
		static int getCentimeterI2CMap(int value);

		/* Callbacks */
		
};

#endif
