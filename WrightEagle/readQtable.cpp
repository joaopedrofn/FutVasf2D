#include<iostream>
#include <fstream>
#include<sstream>

int main(){
    std::stringstream qTableString;
	qTableString << "qTable" << 10;
	std::ifstream qTableFileIn(qTableString.str(), std::ios::binary);
	double qTable[648][10];
	qTableFileIn.read((char *)&qTable, sizeof(qTable));
	qTableFileIn.close();

    for(int i =0; i<648; i++)
        std::cout << i << " -> " << qTable[i][9] << std::endl;

    // std::ofstream qTableFileOut(qTableString.str(), std::ios::binary);
	// qTableFileOut.write((char *)&qTable, sizeof(qTable));
	// qTableFileOut.close();
}
