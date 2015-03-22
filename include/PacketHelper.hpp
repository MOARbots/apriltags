#include <inttypes.h>
#include <cstring>
using namespace std;

#define HEAD uint64_t(63)
#define PACKET_LEN 5 // Number of bytes in each packet
#define HEAD_LENGTH 6
#define ID_LENGTH 5
#define Y_LENGTH 10
#define X_LENGTH 10
#define R_LENGTH 9

/*	We construct the message according to the format
*	111111		6 bit header, pads out total message size to a round number of bytes
*	-----		5 bit ID (max 31 decimal)
*	----------	10 bit y position (max 1023 rounded to the nearest pixel in decimal)
*	----------	10 bit x position (max 1023 rounded to the nearest pixel in decimal)
*	---------	9 bit rotation (max 512 rounded to the nearest degree in decimal)
*/

class BinPacket {
	public:
		uint64_t IDint;
		uint64_t Yint;
		uint64_t Xint;
		uint64_t myRint;
		uint64_t packet;

		void Build() {
			packet = 0;
			packet = packet | myRint;
			packet = packet | (Xint << R_LENGTH);
			packet = packet | (Yint << (R_LENGTH + X_LENGTH));
			packet = packet | (IDint << (R_LENGTH + X_LENGTH + Y_LENGTH));
			packet = packet | (HEAD << (R_LENGTH + X_LENGTH + Y_LENGTH + ID_LENGTH));
			packet = packet << (64 - PACKET_LEN * 8);
		};
};
