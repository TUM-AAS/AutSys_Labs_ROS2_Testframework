#include <gtest/gtest.h>
#include <fstream>
#include <string>
#include <eigen3/Eigen/Core>
#include <iostream>

class TestSuite : public ::testing::Test {
public:
    std::string getNthLine(std::string filename, int lineNumber) {
        std::ifstream results(filename);
        EXPECT_TRUE(results.is_open());
        std::string line;
        for(int i = 0; i < lineNumber; i++) {
            std::getline(results, line);
        }
        return line;
    }

    double getNumberOfString(std::string line) {
        auto posStart = line.find(": ") + 2;
        auto posStop = line.find("\n");
        EXPECT_NE(posStart, std::string::npos);
        std::string numberStr = line.substr(posStart, posStop);
        double number = std::stod(numberStr);
        return number;
    }
    
    Eigen::Vector4d getVectorOfString(std::string line) {
    	Eigen::Vector4d vec;
		auto posStart = line.find(": ") + 2;
		auto posStop = line.find("\n");
		EXPECT_NE(posStart, std::string::npos);
		std::stringstream vectorStr;
		vectorStr << line.substr(posStart, posStop);
		for(int i = 0; i < 4; i++)  {
			vectorStr >> vec[i];
        }
        return vec;
    }
};
TEST_F(TestSuite, checkReportedCalls)  {
	std::cout << "Checks weather enough calls for a position comparison were sent\n";
    std::string line = getNthLine("results.txt", 3);
    int numberCalls = (int)getNumberOfString(line);
    EXPECT_GT(numberCalls, 100);
}

TEST_F(TestSuite, checkAverageDronePosition) {
	std::cout << "Compares the average deviation of the drone position to the desired position\n";
    std::string line = getNthLine("results.txt", 4);
    double averageDeviation = getNumberOfString(line);
    EXPECT_LE(averageDeviation, 0.25);
}
TEST_F(TestSuite, checkMaxDronePosition) {
	std::cout << "Checks if the maximum deviation of the reported drone positions to the desired position is small enough\n";
    std::string line = getNthLine("results.txt", 5);
    double maxDeviation = getNumberOfString(line);
    EXPECT_LE(maxDeviation, 0.5);
}
TEST_F(TestSuite, checkNumberOfFalseDronePositions) {
	std::cout << "Checks if maximal 1 reported drone position has a deviation greater than 0.1 to the desired position\n";
    std::string line = getNthLine("results.txt", 6);
    int numberFalsePositions = (int)getNumberOfString(line);
    EXPECT_EQ(numberFalsePositions, 0);
}
TEST_F(TestSuite, checkRotorSpeedCalls) {
	std::cout << "Checks weather enough calls for a rotor speed comparison were sent\n";
    std::string line = getNthLine("results.txt", 7);
    unsigned long numberCallsRotorSpeeds = (unsigned long)getNumberOfString(line);
    line = getNthLine("results.txt", 3);
    int numberCalls = (int)getNumberOfString(line);
    EXPECT_GT(numberCallsRotorSpeeds, numberCalls*15.0);
}
TEST_F(TestSuite, checkAverageWrench) {
	std::cout << "Checks if the average of all reported wrenches are as expected\n"; 
    std::string line = getNthLine("results.txt", 8);
    Eigen::Vector4d averageWrench = getVectorOfString(line);
    Eigen::Vector4d maxAverageWrench(12.0, 0.01, 0.01, 0.05);
    Eigen::Vector4d minAverageWrench(9.81, -0.05, -0.12, -0.05);
    for(int i = 0; i < 4; i++) {
		EXPECT_GE(averageWrench[i], minAverageWrench[i]);
		EXPECT_LE(averageWrench[i], maxAverageWrench[i]);
    }

}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
