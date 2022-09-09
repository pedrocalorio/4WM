#include "DataStream.h"

#include <string>
#include <fstream>
#include <vector>
#include <stdexcept> // std::runtime_error
#include <sstream> // std::stringstream


  Eigen::VectorXd DataStream::read_csv_column_data(const std::string& fileName, ColumnNumber columnNumber)
  {
    std::vector<double> xDataOutput, yDataOutput;

    // Create an input filestream
    std::ifstream myFile(fileName);

    // Make sure the file is open
    if (!myFile.is_open()) {
      throw std::runtime_error("Could not open the file!");
    }

    // Helper vars
    std::string line;
    double      xData, yData;
    char        comma;

    // Read data, line by line
    while (std::getline(myFile, line)) {

      // Create a stringstream of the current line
      std::stringstream ss(line);

      // Extract each element from the ss
      // Since only double, char, double is expected,
      // if the first row contains the names of columns,
      // it will be ignored.
      while (ss >> xData >> comma >> yData) {

        xDataOutput.push_back(xData);
        yDataOutput.push_back(yData);
      }
    }

    // Close file
    myFile.close();

    if (columnNumber == column_one) {
      //Eigen::VectorXd output(xDataOutput.data(), xDataOutput.data() + xDataOutput.size());
      Eigen::Map<Eigen::VectorXd> output(&xDataOutput[0], xDataOutput.size());
      return output;
    }
    Eigen::Map<Eigen::VectorXd> output(&yDataOutput[0], yDataOutput.size());
    return output;
  }



