#ifndef Ping_h
#define Ping_h

class Ping
{
public:
	Ping(int pin);
	~Ping();
	long readDistance();

private:
	int pingPin;
	long microsecondsToInches(long microseconds);
	long microsecondsToCentimeters(long microseconds);
};
#endif

