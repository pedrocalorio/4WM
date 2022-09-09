#pragma once
#include <string>
#include "Eigen/Dense"



  enum ColumnNumber
  {
    column_one,
    column_two
  };


  class DataStream
  {
  public:
    /// @brief This function will read the first or second column of a .csv file and convert it to
    /// an Eigen::VectorXd. The strings in the first row of the column will be ignored.
    /// @param fileName Name of the .csv file.
    /// @param columnNumber Use the ColumnNumber enumerator to select the column number.
    /// @return An Eigen::VectorXd that contains the numeric values of the selected column.
    static Eigen::VectorXd read_csv_column_data(const std::string& fileName, ColumnNumber columnNumber);
    

  };

