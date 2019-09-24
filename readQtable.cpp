#include<iostream.h>
#include <fstream>

int main(){
    std::stringstream qTableString;
	qTableString << "qTable" << 2;
	std::ifstream qTableFileIn(qTableString.str(), std::ios::binary);
	double qTable[648][10];
	qTableFileIn.read((char *)&qTable, sizeof(qTable));
	qTableFileIn.close();
}
