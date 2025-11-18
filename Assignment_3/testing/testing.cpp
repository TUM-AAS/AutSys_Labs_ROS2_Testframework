#include <gtest/gtest.h>
#include <fstream>
#include <string>
#include <eigen3/Eigen/Core>
#include <iostream>
#include <vector>
#include <algorithm>

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
    
    std::vector<double> getVectorOfString(std::string line) {
    	std::vector<double> vec;
		auto posStart = line.find(": ") + 2;
		auto posStop = line.find("\n");
		EXPECT_NE(posStart, std::string::npos);
        line = line.substr(posStart, posStop);
        line.erase(remove(line.begin(), line.end(), ','), line.end());
        std::stringstream vectorStr(line);
        int count = std::count(line.begin(), line.end(), ' ');
        for(int i = 0; i <= count; i++) {
            double tmp;
            vectorStr >> tmp;
            vec.push_back(tmp);
        }
        return vec;
    }

    Eigen::Vector3d getPositionOfString(std::string line) {
        Eigen::Vector3d pos;
        auto posStart = line.find("[");
		auto posStop = line.find("]");
		EXPECT_NE(posStart, std::string::npos);
        EXPECT_NE(posStop, std::string::npos);
        line = line.substr(posStart+1, posStop-2);
        line.erase(remove(line.begin(), line.end(), ','), line.end());
        std::stringstream vectorStr(line);
        for(int i = 0; i < 3; i++) {
            double tmp;
            vectorStr >> tmp;
            pos[i] = tmp;
        }
        return pos;
    }

    int getGoalNumber() {
        std::string line = getNthLine(filename, 8);
        std::vector<double> goalSequence = getVectorOfString(line);
        int numberGoals = (int)*max_element(std::begin(goalSequence), std::end(goalSequence));
        return numberGoals;
    }

    const std::string filename = "results.txt";
};

TEST_F(TestSuite, checkFlightTime) {
    std::cout << "checks the duration of the recorded flight\n";
    std::string line = getNthLine(filename, 3);
    double flightTime = getNumberOfString(line);
    EXPECT_GT(flightTime, 50);
}

TEST_F(TestSuite, checkReportedCalls)  {
	std::cout << "Checks weather enough calls for a position comparison were sent\n";
    std::string line = getNthLine(filename, 4);
    int numberCalls = (int)getNumberOfString(line);
    EXPECT_GT(numberCalls, 100);
}

TEST_F(TestSuite, checkReportedRotorSpeedCalls) {
	std::cout << "Checks weather enough calls for a rotor speed comparison were sent\n";
    std::string line = getNthLine(filename, 5);
    unsigned long numberCallsRotorSpeeds = (unsigned long)getNumberOfString(line);
    line = getNthLine(filename, 3);
    int numberCalls = (int)getNumberOfString(line);
    EXPECT_GT(numberCallsRotorSpeeds, numberCalls*15.0);
}

TEST_F(TestSuite, checkMaximumWrench) {
	std::cout << "Checks if the maximum of all reported wrenches are as expected\n"; 
    std::string line = getNthLine(filename, 6);
    std::vector<double> maxWrench = getVectorOfString(line);
    Eigen::Vector4d maximalExpectedWrench(30.0, 2.25, 5.0, 2.0);
    for(int i = 0; i < 4; i++) {
		EXPECT_LE(maxWrench[i], maximalExpectedWrench[i]);
    }

}

TEST_F(TestSuite, checkMinimumWrench) {
	std::cout << "Checks if the minimum of all reported wrenches are as expected\n"; 
    std::string line = getNthLine(filename, 7);
    std::vector<double> minWrench = getVectorOfString(line);
    Eigen::Vector4d minimalExpectedWrench(0.0, -2.25, -5.0, -2.0);
    for(int i = 0; i < 4; i++) {
		EXPECT_GE(minWrench[i], minimalExpectedWrench[i]);
    }
}

TEST_F(TestSuite, checkGoalSequence) {
    std::cout << "Checks if the drone flew in the right sequence through the goals\n";
    std::string line = getNthLine(filename, 8);
    std::vector<double> goalSequence = getVectorOfString(line);
    int lastGoal = (int)*max_element(std::begin(goalSequence), std::end(goalSequence));
    EXPECT_EQ(goalSequence.size(), lastGoal*2);
    EXPECT_EQ((int)goalSequence[0], 1);
    for(int i = 0; i < goalSequence.size()-1; i++) {
        if((int)goalSequence[i] != lastGoal) {
            EXPECT_EQ((int)goalSequence[i]+1, (int)goalSequence[i+1]);
        }
        else {
            EXPECT_EQ((int)goalSequence[i]+1-lastGoal, (int)goalSequence[i+1]);
        }
    }
}

TEST_F(TestSuite, checkNumberOfTransitions) {
    std::cout << "Checks weather each goal was passed exactly 2 times\n";
    int numberGoals = getGoalNumber();
    EXPECT_GT(numberGoals, 0);
    for(int i = 0; i < numberGoals; i++) {
        std::string line = getNthLine(filename, 10+i);
        int transitionNumber = (int)getNumberOfString(line);
        EXPECT_EQ(transitionNumber, 2);
    }
}

TEST_F(TestSuite, checkStopPositions) {
    std::cout << "Checks if the drone stopped in the tunnel in the second round\n";

    int numberGoals = getGoalNumber();
    int startingLine = 10 + numberGoals;
    std::string line = getNthLine(filename, startingLine);
    int stopPositions = getNumberOfString(line);
    line = getNthLine(filename, 3);
    double flightTime = getNumberOfString(line);

    int reportedStopsInTunnel = 0;
    double lastStopTime = 0.0;
    for(int i = 0; i < stopPositions; i++) {
        line = getNthLine(filename, startingLine+1+i);
        double time = getNumberOfString(line);
        Eigen::Vector3d tunnelEntrance(16.0, 39.0, 11.0);
        if(time > 2.0/3.0*flightTime) {
            Eigen::Vector3d pos = getPositionOfString(line);
            bool droneInTunnel = (pos[0] < 15.5) && (pos[0] > 10.5) && (abs(pos[1]-tunnelEntrance[1]) < 0.2) && (abs(pos[2]-tunnelEntrance[2]) < 0.2);
            if(droneInTunnel && time-lastStopTime > 0.5)
                reportedStopsInTunnel++;
        }
        
    }
    EXPECT_EQ(reportedStopsInTunnel, 1);
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
